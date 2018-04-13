function generate_roslog(launchfile)
    local script = string.format([[
        logfile="%s"
        rm "$logfile"
        (timeout 5 roslaunch "%s")&
        sleep 2
        for n in `rosnode list` ; do
            rosnode info $n >> $logfile
        done
        sleep 3]], "/tmp/roslog.log", launchfile)
    os.execute(script);    
end

function output_annotation(nodes)
    for i, node in ipairs(nodes) do
        io.stdout:write(node.name, ":\n")
        
        io.stdout:write('<!--\n')
        io.stdout:write('<topics>\n')
        for _, pub in ipairs(node.pub) do
            io.stdout:write('    <topic name="', pub.topic, '"\n')
            io.stdout:write('           type="', pub.type, '"\n')
            io.stdout:write('           class="pub"/>\n')
        end
        for _, sub in ipairs(node.sub) do
            io.stdout:write('    <topic name="', sub.topic, '"\n')
            io.stdout:write('           type="', sub.type, '"\n')
            io.stdout:write('           class="sub"/>\n')
        end
        io.stdout:write('</topics>\n')
        
        io.stdout:write('<services>\n')
        for _, adv in ipairs(node.adv) do
            io.stdout:write('    <service name="', adv.topic, '"\n')
            io.stdout:write('             type="', adv.type or "?", '"\n')
            io.stdout:write('             class="adv"/>\n')
        end
        io.stdout:write('</services>')
        io.stdout:write('-->\n')
    end
end

generate_roslog(arg[1])

local nodes = {}
local logfile = io.open("/tmp/roslog.log", "r")
local line = logfile:read('*line')
local current_node = {pub = {}, sub = {}, adv = {}}

while line do
    local name = line:match('Node %[([^%]]+)%]')
    if name then
        if current_node.name then
            nodes[#nodes + 1] = current_node
        end
        current_node = {pub = {}, sub = {}, adv = {}}
        current_node.name = name
    end

    if line:find('Publications') then
        current_node.section = 'pub'
    end

    if line:find('Subscriptions') then
        current_node.section = 'sub'
    end

    if line:find('Services') then
        current_node.section = 'adv'
    end

    if line == "" then
        current_node.section = nil
    end

    local topic, ttype = line:match(' %* ([^%s]+) %[([^%]]+)%]')
    topic = topic or line:match(' %* ([^%s]+)')
    if topic and current_node.section then
        local s = current_node[current_node.section]
        s[#s + 1] = {topic = topic, type = ttype}
    end

    line = logfile:read('*line')
end
nodes[#nodes + 1] = current_node

logfile:close()
output_annotation(nodes)
