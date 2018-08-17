function output_dot(nodes)
    io.stdout:write('digraph {\n')
    for i, node in ipairs(nodes) do
        io.stdout:write('node', i, '[label="', node.name, '"; shape=box];\n')
        
                io.stdout:write('node', i, ' -> {')
        for idx, pub in ipairs(node.pub) do
            io.stdout:write( idx == 1 and "" or "," ,'"', pub.topic, '"')
        end
        io.stdout:write('};\n')
        
        if #node.adv > 0 then
            io.stdout:write('node', i, ' -> {')
            for idx, adv in ipairs(node.adv) do
                io.stdout:write( idx == 1 and "" or "," ,'"', adv.topic, '"')
            end
            io.stdout:write('[shape=octagon]};\n')
        end
        
        for _, sub in ipairs(node.sub) do
            io.stdout:write('"', sub.topic, '" -> node', i, ';\n')
        end
    end
    io.stdout:write('}\n')
end

local nodes = {}
local line = io.stdin:read('*line')
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

    line = io.stdin:read('*line')
end
nodes[#nodes + 1] = current_node

output_dot(nodes)
