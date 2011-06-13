-- rule types
-- the ones that execute a schema as its effect
SCHEMA_RULE = 0
-- the ones that change pet feelings as its effect
FEELING_RULE = 1
-- the ones that change pet relations as its effect 
RELATION_RULE = 2

-- split the string according to the given patterns
function split(str, pat)
    local t = {}
    local fpat = "(.-)" .. pat
    local last_end = 1
    local s, e, cap = str:find(fpat, 1)

    while s do
        if s ~= 1 or cap ~= "" then
            table.insert(t, cap)
        end
        last_end = e + 1
        s, e, cap = str:find(fpat, last_end)
    end
    
    if last_end <= #str then
        cap = str:sub(last_end)
        table.insert(t, cap)
    end

    return t
end

-- trim the given string, i.e., remove the leading and ending
-- white spaces
function trim(str)
    return (string.gsub(str, "^%s*(.-)%s*$", "%1"))
end

-- add rules 
function rule( name, type_t, modesStrength, precondition, effect )

    -- get the effect name, i.e., substring from the first position till the
    -- first occurance of ( character.
    local effectName = string.sub(effect, 1, (string.find(effect, "%(") - 1))
 
    local values = StringVector()

    -- get the text within the most external enclosing parentesis (included)
    local effectStringParam = string.sub(effect, string.find(effect, "%(.*%)"))

    if(string.len(effectStringParam) > 2) then

        effectStringParam = string.sub(effectStringParam, 2, string.len(effectStringParam) - 1)

        -- not an empty parentesis ( ) 
        if effectStringParam ~= " " then
              
            -- split parameters string
            local effectParams = split(effectStringParam, "w*,w*")

            -- add all to string vector (removing single quotes)
            for k, v in pairs(effectParams) do
                values:push_back(string.format("%s", string.gsub(trim(v), "'", "")))
            end

        end
    end
 
    local modesStrengthMap = StringFloatMap( )
    for mode, strength in pairs(modesStrength) do
       modesStrengthMap:insert( StringFloatPair( mode, strength ) )
    end
 
    ruleEngine:addRule(name, type_t, modesStrengthMap, precondition, effectName, values)
end

