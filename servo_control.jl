# 舵机调试统一入口：只使用这一个文件。
# 终端运行：julia --project=. servo_control.jl [ports|diagnose|hold|turn350|sixturn]
# 默认仅列出 COM 口。diagnose 只读；其余动作会发送控制指令。
# Julia REPL：include("servo_control.jl") 后调用文件末尾的示例函数。
# 舵机控制函数的第一个参数是目标 ID；省略时默认为 1。
using LibSerialPort

function list_com_ports()
    Sys.iswindows() || error("此脚本仅支持 Windows。")
    ports = String[]

    for n in 1:256
        name = "COM$n"
        buffer = zeros(UInt16, 4096)
        count = ccall(
            (:QueryDosDeviceW, "kernel32"), UInt32,
            (Cwstring, Ptr{UInt16}, UInt32),
            name, buffer, length(buffer),
        )
        if count > 0
            push!(ports, name)
            terminator = findfirst(iszero, buffer)
            target = transcode(String, buffer[1:terminator-1])
            println(name, " => ", target)
        end
    end

    isempty(ports) && println("未检测到 COM 口。")
    return ports
end

function validate_servo_id(id::Integer)
    0 <= id <= 253 || error("舵机 ID 必须在 0～253 之间。")
    return id
end

# 查询某个主 ID 是否有应答；仅把明确的应答超时视为不存在。
function servo_responds(sp, id)
    try
        exchange(sp, id, 0x01; timeout=0.35)
        return true
    catch err
        if err isa ErrorException && occursin("应答超时", err.msg)
            return false
        end
        rethrow()
    end
end

"""
    change_servo_id(old_id, new_id)

通过第二个 COM 口修改舵机主 ID。每次仅连接一只待改 ID 的舵机；
新 ID 必须空闲。函数先解锁 EPROM，再修改地址 0x05，读回确认后恢复原锁状态。
"""
function change_servo_id(old_id::Integer, new_id::Integer)
    validate_servo_id(old_id)
    validate_servo_id(new_id)
    old_id != new_id || error("新旧 ID 相同，无需修改。")
    ports = list_com_ports()
    length(ports) >= 2 || error("未找到第二个 COM 口。")
    sp = LibSerialPort.open(ports[2], 1_000_000)
    LibSerialPort.set_write_timeout(sp, 2.0)
    lock_may_be_open = false
    original_lock = nothing
    active_id = old_id
    try
        exchange(sp, old_id, 0x01)
        stored_id = readreg(sp, old_id, 0x05, 1)[1]
        stored_id == old_id || error("读取到的主 ID 是 $stored_id ，与旧 ID 不符。")
        servo_responds(sp, new_id) && error("新 ID $(new_id) 已有舵机应答，停止修改。")

        original_lock = readreg(sp, old_id, 0x37, 1)[1]
        lock_may_be_open = true
        writereg(sp, old_id, 0x37, UInt8[0])
        readreg(sp, old_id, 0x37, 1)[1] == 0 || error("EPROM 未成功解锁。")

        # ID 写入后应答可能使用旧或新 ID，因此直接发写包，再用新 ID 查询确认。
        write(sp, packet(old_id, 0x03, UInt8[0x05, UInt8(new_id)]))
        sleep(0.15)
        servo_responds(sp, new_id) || error("新 ID $(new_id) 未应答；请检查舵机当前 ID。")
        active_id = new_id
        readreg(sp, new_id, 0x05, 1)[1] == new_id || error("新 ID 写入后读回不一致。")
        writereg(sp, new_id, 0x37, UInt8[original_lock])
        readreg(sp, new_id, 0x37, 1)[1] == original_lock || error("原 EPROM 锁状态未恢复。")
        lock_may_be_open = false
        println("主 ID 已由 $(old_id) 修改为 $(new_id)；请断电重上电后再次查询新 ID。")
        return new_id
    finally
        if lock_may_be_open && original_lock !== nothing
            restored = false
            for candidate in (active_id, active_id == old_id ? new_id : old_id)
                try
                    if servo_responds(sp, candidate)
                        writereg(sp, candidate, 0x37, UInt8[original_lock])
                        restored = readreg(sp, candidate, 0x37, 1)[1] == original_lock
                        restored && break
                    end
                catch
                    # 尝试另一个可能仍在使用的 ID。
                end
            end
            restored || @warn "无法确认 EPROM 锁已恢复，请在继续写配置前检查。"
        end
        close(sp)
    end
end

function packet(id, instruction, params=UInt8[])
    body = UInt8[id, length(params) + 2, instruction, params...]
    return UInt8[0xff, 0xff, body..., (~sum(Int.(body))) & 0xff]
end

function exchange(sp, id, instruction, params=UInt8[]; timeout=0.8)
    n = bytesavailable(sp)
    n > 0 && read(sp, n)
    write(sp, packet(id, instruction, params))
    data = UInt8[]
    deadline = time() + timeout
    while time() < deadline
        n = bytesavailable(sp)
        n > 0 && append!(data, read(sp, n))
        while length(data) >= 4
            if data[1:2] != UInt8[0xff, 0xff]
                popfirst!(data)
                continue
            end
            len = Int(data[4])
            if len < 2
                popfirst!(data)
                continue
            end
            total = len + 4
            length(data) < total && break
            frame = splice!(data, 1:total)
            frame[3] == id || continue
            (sum(Int.(frame[3:end])) & 0xff) == 0xff || error("应答校验失败")
            frame[5] == 0 || error("舵机错误状态：$(frame[5])")
            return frame[6:end-1]
        end
        sleep(0.005)
    end
    error("ID $id 应答超时；检查供电、波特率、ID 和控制板是否支持协议透传。")
end

function readreg(sp, id, address, count)
    data = exchange(sp, id, 0x02, UInt8[address, count])
    length(data) == count || error("应答数据长度不正确")
    return data
end
writereg(sp, id, address, data) = exchange(sp, id, 0x03, UInt8[address, data...])
function position(sp, id)
    data = readreg(sp, id, 0x38, 2)
    raw = Int(data[1]) | (Int(data[2]) << 8)
    return (raw & 0x8000) == 0 ? raw : -(raw & 0x7fff)
end
function target!(sp, id, value)
    -32767 <= value <= 32767 || error("目标位置超出舵机支持范围。")
    raw = abs(value) | (value < 0 ? 0x8000 : 0)
    writereg(sp, id, 0x2a, UInt8[raw & 0xff, (raw >> 8) & 0xff])
end

function set_angle_limits!(sp, id, data)
    writereg(sp, id, 0x09, data)
    readreg(sp, id, 0x09, 4) == data || error("角度限位写入后校验失败。")
end

function turn_350_degrees(id::Integer=1; speed_rpm::Real=15.0)
    validate_servo_id(id)
    isfinite(speed_rpm) && 0 < speed_rpm <= 113 ||
        error("速度须大于 0，且不超过规格书中 7.4 V 空载速度 113 RPM。")
    speed_units = round(Int, speed_rpm / 0.732)
    speed_units >= 1 || error("速度低于一个寄存器单位（0.732 RPM）。")
    ports = list_com_ports()
    length(ports) >= 2 || error("至少需要两个 COM 口，当前为 $(ports)；未发送运动指令。")
    port = ports[2]
    println("选择控制板：$(port)；舵机 ID：$(id)；波特率：1000000")
    sp = LibSerialPort.open(port, 1_000_000)
    LibSerialPort.set_write_timeout(sp, 1.0)
    original_lock = nothing
    original_limits = nothing
    limits_changed = false
    started = false
    try
        exchange(sp, id, 0x01)
        println("PING 成功")
        readreg(sp, id, 0x08, 1)[1] == 1 || error("应答级别不是 1；本测试需要写指令应答。")
        mode = readreg(sp, id, 0x21, 1)[1]
        mode in (0, 4) || error("当前运行模式 $mode 不是位置模式，停止测试。")
        original_lock = readreg(sp, id, 0x37, 1)[1]
        start = position(sp, id)
        original_limits = readreg(sp, id, 0x09, 4)
        minimum = Int(original_limits[1]) | (Int(original_limits[2]) << 8)
        maximum = Int(original_limits[3]) | (Int(original_limits[4]) << 8)
        (original_limits == UInt8[0, 0, 0, 0] || minimum <= start <= maximum) ||
            error("当前位置不在原角度限位内。")
        travel = round(Int, 350 / 360 * 4096)
        goal = start + travel
        goal <= 32767 || error("多圈目标位置超出舵机支持范围。")
        println("位置模式：$(mode)；初始位置：$(start)；测试目标：$(goal)（相对移动约 350°）；速度上限约 $(speed_units * 0.732) RPM")
        started = true
        target!(sp, id, start)
        writereg(sp, id, 0x28, UInt8[0])
        if original_limits != UInt8[0, 0, 0, 0]
            writereg(sp, id, 0x37, UInt8[0])
            limits_changed = true
            set_angle_limits!(sp, id, UInt8[0, 0, 0, 0])
            writereg(sp, id, 0x37, UInt8[original_lock])
        end
        abs(position(sp, id) - start) <= 20 || error("切换多圈模式后位置读数变化，停止运动。")
        writereg(sp, id, 0x2e, UInt8[speed_units & 0xff, (speed_units >> 8) & 0xff])
        target!(sp, id, start)
        writereg(sp, id, 0x28, UInt8[1])
        for endpoint in (goal, start)
            actual = position(sp, id)
            distance_turns = abs(endpoint - actual) / TICKS_PER_TURN
            timeout = max(20.0, 60 * distance_turns / speed_rpm * 2.5 + 10)
            target!(sp, id, endpoint)
            wait_for_position(sp, id, endpoint; timeout)
        end
        println("约 350° 往返测试完成。")
    finally
        if started
            try
                writereg(sp, id, 0x28, UInt8[0])
                current = position(sp, id)
                lower = Int(original_limits[1]) | (Int(original_limits[2]) << 8)
                upper = Int(original_limits[3]) | (Int(original_limits[4]) << 8)
                in_old_limits = original_limits == UInt8[0, 0, 0, 0] || lower <= current <= upper
                desired_limits = in_old_limits ? original_limits : UInt8[0, 0, 0, 0]
                if readreg(sp, id, 0x09, 4) != desired_limits
                    writereg(sp, id, 0x37, UInt8[0])
                    set_angle_limits!(sp, id, desired_limits)
                    writereg(sp, id, 0x37, UInt8[original_lock])
                end
                target!(sp, id, current)
                writereg(sp, id, 0x28, UInt8[1])
                println("保持当前位置 $(current)；扭矩开关已设为 1。")
                if !in_old_limits
                    @warn "当前位置在原单圈限位外，暂保留多圈限位。"
                elseif limits_changed
                    println("原角度限位已恢复。")
                end
            catch err
                @warn "无法确认已保持当前位置，请读取扭矩开关与目标位置" exception=err
            end
        end
        close(sp)
    end
end

function diagnose(id::Integer=1)
    validate_servo_id(id)
    ports = list_com_ports()
    length(ports) >= 2 || error("未找到第二个 COM 口。")
    port = ports[2]
    println("只读诊断，控制板：$(port)，ID：$(id)")
    for baud in (1_000_000, 115_200, 500_000, 250_000, 128_000, 76_800, 57_600, 38_400)
        println("查询波特率：$baud")
        sp = LibSerialPort.open(port, baud)
        try
            LibSerialPort.set_write_timeout(sp, 1.0)
            sleep(0.15)
            exchange(sp, id, 0x01; timeout=0.5)
            println("ID $(id) 有效应答，波特率：$baud")
            println("当前位置：", position(sp, id))
            println("运行模式：", readreg(sp, id, 0x21, 1)[1])
            println("电压：", readreg(sp, id, 0x3e, 1)[1] / 10, " V")
            return baud
        catch err
            println(sprint(showerror, err))
        finally
            close(sp)
        end
    end
    println("所有波特率均未取得有效应答；未发送运动指令。")
    return nothing
end

const TICKS_PER_TURN = 4096
const TEST_TURNS = 6
const TEST_CYCLES = 3
const SPEED_RPM = 15.0

u16(data) = Int(data[1]) | (Int(data[2]) << 8)
le16(value) = UInt8[value & 0xff, (value >> 8) & 0xff]

function wait_for_position(sp, id, goal; timeout)
    started = time()
    last_report = started
    while time() - started < timeout
        actual = position(sp, id)
        if abs(actual - goal) <= 30
            println("到达目标 $(goal)；反馈 $(actual)；用时 $(round(time()-started, digits=1)) 秒")
            return
        end
        if time() - last_report >= 3
            println("目标 $(goal)；反馈 $(actual)；剩余 $(goal-actual) 步")
            last_report = time()
        end
        sleep(0.2)
    end
    error("等待目标 $goal 超时；当前位置 $(position(sp, id))")
end

function main_six_turns(id::Integer=1)
    validate_servo_id(id)
    ports = list_com_ports()
    length(ports) >= 2 || error("未找到第二个 COM 口；未发送运动指令。")
    port = ports[2]
    println("控制板 $(port)，舵机 ID $(id)，三周期 ±6 圈测试")
    sp = LibSerialPort.open(port, 1_000_000)
    LibSerialPort.set_write_timeout(sp, 1.0)
    old_limits = nothing
    old_lock = nothing
    changed_limits = false
    touched_torque = false
    try
        exchange(sp, id, 0x01)
        readreg(sp, id, 0x08, 1)[1] == 1 || error("舵机写指令应答未启用。")
        mode = readreg(sp, id, 0x21, 1)[1]
        mode in (0, 4) || error("当前模式 $mode 不是位置控制模式。")
        old_limits = readreg(sp, id, 0x09, 4)
        old_lock = readreg(sp, id, 0x37, 1)[1]
        center = position(sp, id)
        excursion = TEST_TURNS * TICKS_PER_TURN
        plus = center + excursion
        minus = center - excursion
        -32767 <= minus && plus <= 32767 || error("±6 圈目标超出位置寄存器范围。")
        speed_units = round(Int, SPEED_RPM / 0.732)
        println("中心 $(center)；正向目标 $(plus)；反向目标 $(minus)；速度上限约 $(SPEED_RPM) RPM")

        # 先关闭扭矩、保持当前目标，再临时设置多圈模式。
        writereg(sp, id, 0x28, UInt8[0])
        touched_torque = true
        target!(sp, id, center)
        if old_limits != UInt8[0, 0, 0, 0]
            writereg(sp, id, 0x37, UInt8[0])
            changed_limits = true
            set_angle_limits!(sp, id, UInt8[0, 0, 0, 0])
            writereg(sp, id, 0x37, UInt8[old_lock])
        end
        abs(position(sp, id) - center) <= 30 || error("切换多圈模式后位置读数变化。")
        writereg(sp, id, 0x2e, le16(speed_units))
        target!(sp, id, center)
        writereg(sp, id, 0x28, UInt8[1])

        for cycle in 1:TEST_CYCLES
            println("第 $(cycle)/$(TEST_CYCLES) 周期")
            for goal in (plus, minus, center)
                actual = position(sp, id)
                distance_turns = abs(goal-actual) / TICKS_PER_TURN
                timeout = max(20.0, 60 * distance_turns / SPEED_RPM * 2.5 + 10)
                target!(sp, id, goal)
                wait_for_position(sp, id, goal; timeout)
            end
        end
        println("三周期 ±6 圈测试完成。")
    finally
        if touched_torque
            try
                writereg(sp, id, 0x28, UInt8[0])
                current = position(sp, id)
                lower = u16(old_limits[1:2])
                upper = u16(old_limits[3:4])
                in_old_limits = old_limits == UInt8[0, 0, 0, 0] || lower <= current <= upper
                desired_limits = in_old_limits ? old_limits : UInt8[0, 0, 0, 0]
                actual_limits = readreg(sp, id, 0x09, 4)
                if actual_limits != desired_limits
                    writereg(sp, id, 0x37, UInt8[0])
                    set_angle_limits!(sp, id, desired_limits)
                    writereg(sp, id, 0x37, UInt8[old_lock])
                end
                target!(sp, id, current)
                writereg(sp, id, 0x28, UInt8[1])
                println("保持当前位置 $(current)；扭矩开关已设为 1。")
                if !in_old_limits
                    @warn "当前位置在原单圈限位外，暂保留多圈限位；回到原范围后再恢复。"
                elseif changed_limits
                    println("原角度限位已恢复。")
                end
            catch err
                @warn "无法确认已保持当前位置，请读取扭矩开关与目标位置" exception=err
            end
        end
        close(sp)
    end
end

function hold_position(id::Integer=1)
    validate_servo_id(id)
    ports = list_com_ports()
    length(ports) >= 2 || error("未找到第二个 COM 口。")
    sp = LibSerialPort.open(ports[2], 1_000_000)
    LibSerialPort.set_write_timeout(sp, 2.0)
    try
        exchange(sp, id, 0x01)
        mode = readreg(sp, id, 0x21, 1)[1]
        mode in (0, 4) || error("当前模式 $mode 不是位置控制模式。")
        current = position(sp, id)
        limits = readreg(sp, id, 0x09, 4)
        minimum = Int(limits[1]) | (Int(limits[2]) << 8)
        maximum = Int(limits[3]) | (Int(limits[4]) << 8)
        if limits != UInt8[0, 0, 0, 0] && !(minimum <= current <= maximum)
            lock = readreg(sp, id, 0x37, 1)[1]
            writereg(sp, id, 0x28, UInt8[0])
            writereg(sp, id, 0x37, UInt8[0])
            try
                set_angle_limits!(sp, id, UInt8[0, 0, 0, 0])
            finally
                writereg(sp, id, 0x37, UInt8[lock])
            end
            println("当前位置超出单圈限位，已切换到多圈限位。")
        end
        target!(sp, id, current)
        writereg(sp, id, 0x28, UInt8[1])
        println("保持位置 $(current)；扭矩开关：", readreg(sp, id, 0x28, 1)[1])
        println("反馈位置：", position(sp, id))
    finally
        close(sp)
    end
end
function run_cli()
    mode = isempty(ARGS) ? "ports" : lowercase(ARGS[1])
    id = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 1
    if mode == "ports"
        list_com_ports()
    elseif mode == "diagnose"
        diagnose(id)
    elseif mode == "hold"
        hold_position(id)
    elseif mode == "turn350"
        speed_rpm = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 15.0
        turn_350_degrees(id; speed_rpm)
    elseif mode == "sixturn"
        main_six_turns(id)
    else
        error("未知模式 $(mode)；可用模式：ports、diagnose、hold、turn350、sixturn。")
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    run_cli()
end

#
# 逐行调用示例（这些是说明文字，加载整个文件时不会自动执行）：

# 先在 Julia REPL 中加载一次：
include("D:/Seminar/Julia/Code/portable_two_segment_pcc/servo_control.jl")

# 随后仅选中所需的函数调用文字并执行“运行选中内容”，或复制该行到 REPL。
# 不要用“运行代码块”执行这一整段。括号内的数字是舵机 ID，省略时默认为 1。

# 列出 COM 口：
list_com_ports()

# 只读诊断 ID 1（换成 2 即诊断 ID 2）：
diagnose(1)

# 让指定舵机保持当前角度：
hold_position(1)

# 从当前位置连续转约 350°，然后返回；speed_rpm 是速度上限，默认 15 RPM：
turn_350_degrees(1; speed_rpm=15)

# 向正负各转 6 圈往返，共 3 个周期：
main_six_turns(1)

# 只接一只待修改舵机时，将主 ID 从 1 改为 2：
change_servo_id(1, 2)
# =#
