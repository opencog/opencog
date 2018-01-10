/*************************************************************************/
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

package org.linkgrammar;

import java.io.Reader;
import java.text.CharacterIterator;
import java.text.StringCharacterIterator;
import java.util.HashMap;
import java.util.Map;

/**
 * Link-grammar server JSON-related utilities.
 *
 * @author Borislav Iordanov
 * @author Linas Vepstas
 *
 */
public class JSONUtils
{
	public static boolean getBool(String name, Map<String, String> msg, boolean def)
	{
		String x = msg.get(name);
		return x == null ? def : Boolean.valueOf(x);
	}

	public static int getInt(String name, Map<String, String> msg, int def)
	{
		String x = msg.get(name);
		return x == null ? def : Integer.parseInt(x);
	}

	public static double getDouble(String name, Map<String, String> msg, double def)
	{
		String x = msg.get(name);
		return x == null ? def : Double.parseDouble(x);
	}

	static char[] hex = "0123456789ABCDEF".toCharArray();

	public static String jsonString(String s)
	{
		if (s == null)
			return null;
		StringBuffer b = new StringBuffer();
		b.append("\"");
		CharacterIterator it = new StringCharacterIterator(s);
		for (char c = it.first(); c != CharacterIterator.DONE; c = it.next())
		{
			if (c == '"') b.append("\\\"");
			else if (c == '\\') b.append("\\\\");
			else if (c == '/') b.append("\\/");
			else if (c == '\b') b.append("\\b");
			else if (c == '\f') b.append("\\f");
			else if (c == '\n') b.append("\\n");
			else if (c == '\r') b.append("\\r");
			else if (c == '\t') b.append("\\t");
			else if (Character.isISOControl(c))
			{
				int n = c;
				for (int i = 0; i < 4; ++i) {
					int digit = (n & 0xf000) >> 12;
					b.append(hex[digit]);
					n <<= 4;
				}
			}
			else
			{
				b.append(c);
			}
		}
		b.append("\"");
		return b.toString();
	}

	private String rawText;
	public String getRawText() { return rawText; }

	public Map<String, String> readMsg(Reader in) throws java.io.IOException
	{
		// Read chars from input until input is exhausted, or until
		// newline is encountered. "length" will be set to the final
		// length of the input.  The char array buf stores the input;
		// it is automatically expanded to handle very long inputs.
		int length = 0;
		char [] buf = new char[1024];
		for (int count = in.read(buf, length, buf.length - length);
		     count > -1;
		     count = in.read(buf, length, buf.length - length))
		{
			length += count;
			if (length == buf.length)
			{
				char [] nbuf = new char[buf.length + 512];
				System.arraycopy(buf, 0, nbuf, 0, buf.length);
				buf = nbuf;
			}
			if (buf[length-1] == '\n')
				break;
		}
		rawText = new String(buf);

		// "result" will contain a map of key-value pairs extracted from
		// the JSON input. (viz, buf is assumed to contain valid json)
		Map<String, String> result = new HashMap<String, String>();

		// Note that we expect the JSON part of 'buf' to be in ASCII.
		// However, the everything after 'text:' might be in UTF-8
		// That's OK, since the code below just assumes ASCII while
		// grepping for 'text:' and then lets Java String constructor
		// deal with the UTF-8. Anyway, it works.
		boolean gotText = false;
		int start = -1;
		int column = -1;
		for (int offset = 0; offset < length - 1; offset++)
		{
			char c = buf[offset];
			if (start == -1)
				start = offset;
			else if (c == ':' && column == -1)
			{
				column = offset;
				String name = new String(buf, start, column - start);
				name = name.trim();
				if ("text".equals(name)) gotText = true;
			}

			// Any commas appearing in the sentence text will fuck this up,
			// so we treat this as a JSON message, but only until the key
			// "text" is seen. After that, all commas are assumed to be part
			// of the text message. (ergo, "text" must the last keyword in
			// the message.) Note also: after that, the rest might be in UTF-8
			else if (c == '\0' || (!gotText && c == ','))
			{
				if (start == -1 || column == -1)
					throw new RuntimeException("Malformed message:" + new String(buf, 0, length));
				String name = new String(buf, start, column - start);
				String value = new String(buf, column + 1, offset - column - 1);
				name = name.trim();
				value = value.trim();
				result.put(name, value);
				start = column = -1;
			}
		}

		// If we are here, the the last byte wasn't null. This is the
		// normal exit, I guess ...
		if (start != -1 && column != -1) {
			String name = new String(buf, start, column - start);
			String value = new String(buf, column + 1, length - column - 1);
			name = name.trim();
			value = value.trim();
			result.put(name, value);
			start = column = -1;
		}
		if (start != -1 || column != -1)
			throw new RuntimeException("Malformed message:"
				+ new String(buf, 0, length)
				+ "Did you forget to say \"text:\" at the start of the message?");
		return result;
	}
}

