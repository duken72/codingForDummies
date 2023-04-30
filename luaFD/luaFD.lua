#!/usr/bin/lua
-- Src: https://learnxinyminutes.com/docs/lua/

-- Single-line comment

--[[
	Multi-lines comment
--]]

----------------------------------------------------
-- 1. Variables and flow control.
----------------------------------------------------
print '\n1. Variables and flow control'
-- Variables are global by default.
NUM = 27
-- Local variable
local num = 25

local s = 'walternate'  -- Immutable strings like Python.
local t = "double-quotes are also fine"
U = [[ Double brackets
       start and end
       multi-line strings.]]
local y = nil  -- Undefines t; Lua has garbage collection.


-- Conditional statements
while num < 50 do
  num = num + 1  -- No ++ or += type operators.
end

if num > 40 then
  print('over 40')
elseif s ~= 'walternate' then  -- ~= not equals, == equals
  io.write('not over 40\n')  -- Defaults to stdout.
else
end

for i = 1, 14 do  -- The range includes both ends.
  i = i + 1
end

repeat
  num = num - 1
until num == 0

-- IO
io.write('write to stdout with io.write\n')  -- Defaults to stdout.
io.write('read from stdin with io.read\n')
-- local line = io.read()  -- Reads next stdin line.
-- String concatenation uses the .. operator:
-- print('Winter is coming, ' .. line)

-- Logical
-- Only nil and false are falsy; 0 and '' are true!
-- 'or' and 'and' are short-circuited.

----------------------------------------------------
-- 2. Functions.
----------------------------------------------------
print '\n2. Functions'
function fib(n)
  if n < 2 then return 1 end
  return fib(n - 2) + fib(n - 1)
end

x, y, z = 1, 2, 3, 4
-- x = 1, y = 2, z = 3, and 4 is thrown away.

function bar(a, b, c)
  print(a, b, c)
  return 4, 8, 15, 16, 23, 42
end

x, y = bar('zaphod')  --> prints "zaphod  nil nil"
-- Now x = 4, y = 8, values 15...42 are discarded.

----------------------------------------------------
-- 3. Tables.
----------------------------------------------------
print '\n3. Tables'
-- Dict literals have string keys by default:
t = {key1 = 'value1', key2 = false}

-- String keys can use js-like dot notation:
print(t.key1)  -- Prints 'value1'.
t.newKey = {}  -- Adds a new key/value pair.
t.key2 = nil   -- Removes key2 from the table.
