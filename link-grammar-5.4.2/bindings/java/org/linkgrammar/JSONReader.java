/*
 * Copyright 2009 Borislav Iordanov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.linkgrammar;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.text.CharacterIterator;
import java.text.StringCharacterIterator;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class JSONReader
{
    private static final Object OBJECT_END = new Object();
    private static final Object ARRAY_END = new Object();
    private static final Object COLON = new Object();
    private static final Object COMMA = new Object();
    public static final int FIRST = 0;
    public static final int CURRENT = 1;
    public static final int NEXT = 2;

    private static Map<Character, Character> escapes = new HashMap<Character, Character>();
    static
    {
        escapes.put(new Character('"'), new Character('"'));
        escapes.put(new Character('\\'), new Character('\\'));
        escapes.put(new Character('/'), new Character('/'));
        escapes.put(new Character('b'), new Character('\b'));
        escapes.put(new Character('f'), new Character('\f'));
        escapes.put(new Character('n'), new Character('\n'));
        escapes.put(new Character('r'), new Character('\r'));
        escapes.put(new Character('t'), new Character('\t'));
    }

    private CharacterIterator it;
    private char c;
//    private Object token;
    private StringBuffer buf = new StringBuffer();

    private void error(String msg)
    {
        throw new RuntimeException("JSON parse error near position " +
                                   it.getIndex() + ":" + msg);
    }

    private char next()
    {
        c = it.next();
        return c;
    }

    private char previous()
    {
        c = it.previous();
        return c;
    }

    private void skipWhiteSpace()
    {
        do
        {
            if (Character.isWhitespace(c))
                ;
            else if (c == '/')
            {
                next();
                if (c == '*')
                {
                    // skip multiline comments
                    while (c != CharacterIterator.DONE)
                        if (next() == '*' && next() == '/')
                            break;
                    if (c == CharacterIterator.DONE)
                        throw new RuntimeException(
                                "Unterminated comment while parsing JSON string.");
                }
                else if (c == '/')
                    while (c != '\n' && c != CharacterIterator.DONE)
                        next();
                else
                {
                    previous();
                    break;
                }
            }
            else
                break;
        } while (next() != CharacterIterator.DONE);
    }

    public Object read(CharacterIterator ci, int start)
    {
        it = ci;
        switch (start)
        {
            case FIRST:
                c = it.first();
                break;
            case CURRENT:
                c = it.current();
                break;
            case NEXT:
                c = it.next();
                break;
        }
        return read();
    }

    public Object read(CharacterIterator it)
    {
        return read(it, NEXT);
    }

    public Object read(String string)
    {
        return read(new StringCharacterIterator(string), FIRST);
    }

    private Object read()
    {
        skipWhiteSpace();
        char ch = c;
        next();
        Object value = null;
        switch (ch)
        {
            case '"':
                value = string();
                break;
            case '[':
                value = array();
                break;
            case ']':
                value = ARRAY_END;
                break;
            case ',':
                value = COMMA;
                break;
            case '{':
                value = object();
                break;
            case '}':
                value = OBJECT_END;
                break;
            case ':':
                value = COLON;
                break;
            case 't':
                if (c != 'r' || next() != 'u' || next() != 'e')
                    error("Invalid JSON token: expected 'true' keyword.");
                next();
                value = Boolean.TRUE;
                break;
            case 'f':
                if (c != 'a' || next() != 'l' || next() != 's' || next() != 'e')
                    error( "Invalid JSON token: expected 'false' keyword.");
                next();
                value = Boolean.FALSE;
                break;
            case 'n':
                if (c != 'u' || next() != 'l' || next() != 'l')
                    error("Invalid JSON token: expected 'null' keyword.");
                next();
                value = null;
                break;
            default:
                c = it.previous();
                if (Character.isDigit(c) || c == '-')
                    value = number();
                else
                    error("Invalid character '" + c + "', expecting a JSON token.");
        }
        return value;
    }

    private boolean isValue(Object x)
    {
        return x == null ||
               x instanceof Boolean ||
               x instanceof String ||
               x instanceof Number ||
               x instanceof Map<?,?> ||
               x instanceof List<?>;
    }

    private Object object()
    {
        Map<Object, Object> ret = new HashMap<Object, Object>();
        Object key = read();
        while (key != OBJECT_END)
        {
            if (! (key instanceof String))
                error("Expecting a string as key in object, but got '" + key + "'");
            Object colon = read();
            if (colon != COLON)
                error("Expecting a colon, but found '" +
                      colon + "' after object key '" + key + "'");
            Object value = read();
            if (!isValue(value))
                error("Unexpected value in object with key '" + key + "' -- '" + value +
                      "', expecting boolean, number string, array or object.");
            ret.put(key, value);
            key = read();
            if (key == COMMA)
                key = read();
            else if (key != OBJECT_END)
                error("Unexpected token in object '" + key + "', expecting comma or }.");
        }
        return ret;
    }

    private Object array()
    {
        List<Object> ret = new ArrayList<Object>();
        Object value = read();
        while (value != ARRAY_END)
        {
            if (!isValue(value))
                error("Unexpected value in array '" + value +
                      "', expecting boolean, number string, array or object.");
            ret.add(value);
            value = read();
            if (value == COMMA)
                value = read();
            else if (value != ARRAY_END)
                error("Unexpected token in array '" +
                      value + "', expecting comma or ].");
        }
        return ret;
    }

    private Object number()
    {
        int length = 0;
        boolean isFloatingPoint = false;
        buf.setLength(0);

        if (c == '-')
        {
            add(c);
        }
        length += addDigits();
        if (c == '.')
        {
            add(c);
            length += addDigits();
            isFloatingPoint = true;
        }
        if (c == 'e' || c == 'E')
        {
            add(c);
            if (c == '+' || c == '-')
            {
                add(c);
            }
            addDigits();
            isFloatingPoint = true;
        }

        String s = buf.toString();
        return isFloatingPoint ? (length < 17) ? (Object) Double.valueOf(s)
                : new BigDecimal(s) : (length < 20) ? (Object) Long.valueOf(s)
                : new BigInteger(s);
    }

    private int addDigits()
    {
        int ret;
        for (ret = 0; Character.isDigit(c); ++ret)
        {
            add(c);
        }
        return ret;
    }

    private Object string()
    {
        buf.setLength(0);
        while (c != '"')
        {
            if (c == '\\')
            {
                next();
                if (c == 'u')
                {
                    add(unicode());
                }
                else
                {
                    Character value = escapes.get(new Character(c));
                    if (value != null)
                        add(value);
                    else
                        add(c);
                }
            }
            else
            {
                add(c);
            }
        }
        next();
        return buf.toString();
    }

    private void add(char cc)
    {
        buf.append(cc);
        next();
    }

    private char unicode()
    {
        int value = 0;
        for (int i = 0; i < 4; ++i)
        {
            switch (next())
            {
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                    value = (value << 4) + c - '0';
                    break;
                case 'a':
                case 'b':
                case 'c':
                case 'd':
                case 'e':
                case 'f':
                    value = (value << 4) + (c - 'a') + 10;
                    break;
                case 'A':
                case 'B':
                case 'C':
                case 'D':
                case 'E':
                case 'F':
                    value = (value << 4) + (c - 'A') + 10;
                    break;
            }
        }
        return (char) value;
    }
}
