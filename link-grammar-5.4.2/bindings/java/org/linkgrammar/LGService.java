/*************************************************************************/
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

package org.linkgrammar;

import java.io.File;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.Reader;
import java.net.ServerSocket;
import java.net.Socket;
import java.text.SimpleDateFormat;
import java.util.Map;
// import java.util.concurrent.Future;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * A simple server implementation for running Link Grammar as a
 * standalone server. The server accepts parsing requests and returns
 * the result as a JSON formatted string (see this
 * <a href="http://www.json.org">JSON</a> website for more information).
 * There is no session maintained between client and server, it's a
 * simple, stateless, single round-trip, request-response protocol.
 *
 * Requests consist of a bag of parameters separated by the null '\0'
 * character. Each request must be terminated with the newline '\n'
 * character. Each parameter has the form <em>name:value\0</em> where
 * <em>name</em> and <em>value</em> can contain any character except
 * '\0' and '\n'. The following parameters are recognized:
 *
 * <ul>
 * <li><b>maxLinkages</b> - maximum number of parses to return in the
 *      result. Note that this does not affect the parser behavior which
 *      computes all parses anyway.</li>
 * <li><b>maxParseSeconds</b> - abort parsing if it takes longer than this</li>
 * <li><b>maxCost</b> - don't report linkages with cost greater than this</li>
 * <li><b>storeConstituentString</b> - whether to return the constituent
 *      string for each Linkage as part of the result.</li>
 * <li><b>storeDiagramString</b> - return ASCII-art diagram.</li>
 * <li><b>storeSense</b> - return word-sense tags.</li>
 * <li><b>text</b> - The text to parse. Note that it must be stripped
 *      from newlines.</li>
 * </ul>
 *
 * The server maintains incoming requests in an unbounded queue and
 * handles them in thread pool whose size can be specified at the
 * command line. A thread pool of size > 1 will only work if the Link
 * Grammar version used is thread-safe.
 *
 * Execute this class as a main program to view a list of options.
 *
 * @author Borislav Iordanov
 *
 */
public class LGService
{
	private static boolean verbose = false;
	private static SimpleDateFormat dateFormatter =
		 new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");

	// Multi-threading:
	//
	// Before thread-exit, the LinkGrammar.close() method should be
	// called -- it frees the per-thread data structures in the JNI
	// LinkGrammar code, specifically, the per-thread sentence and
	// linkage data structures. A mem leak will result if this is not
	// called before thread exit.
	//
	// Upon LinkGrammar class destruction: the LinkGrammar.doFinalize()
	// method should be called -- it frees the global data structures
	// (specifically, the dictionary, shared by all threads).
	//
	private static ThreadLocal<Boolean> initialized = new ThreadLocal<Boolean>()
	{ protected Boolean initialValue() { return Boolean.FALSE; } };

	/**
	 * <p>Return <code>true</code> if LinkGrammar is initialized for
	 * the current thread and <code>false</code> otherwise.
	 */
	public static boolean isInitialized()
	{
		return initialized.get();
	}

	/**
	 * <p>
	 * Initialize LinkGrammar for the current thread, if this is not
	 * already done. Note that this method is called by all other
	 * methods in this class that invoke LinkGrammar so there's no
	 * real need to call it yourself. It is safe to call the method
	 * repeatedly. Note that the dictionary language/location must be
	 * set *before* calling init.
	 * </p>
	 */
	public static void init()
	{
		if (!initialized.get())
		{
			LinkGrammar.init();
			initialized.set(Boolean.TRUE);
		}
	}

	/**
	 * <p>
	 * Cleanup allocated memory for use of LinkGrammar in the current thread.
	 * </p>
	 */
	public static void close()
	{
		LinkGrammar.close();
		initialized.set(Boolean.FALSE);
	}
	public static void doFinalize()
	{
		LinkGrammar.doFinalize();
	}

	private static void trace(String s)
	{
		if (verbose)
			System.out.println("LG " + dateFormatter.format(new java.util.Date()) + " " + s);
	}

	/**
	 * <p>
	 * Apply configuration parameters to the parser.
	 * </p>
	 */
	public static void configure(LGConfig config)
	{
		init();

		if (config.getMaxCost() > -1.0)
			LinkGrammar.setMaxCost(config.getMaxCost());
		if (config.getMaxParseSeconds() > -1)
			LinkGrammar.setMaxParseSeconds(config.getMaxParseSeconds());

		// XXX DO NOT DO THIS!!! This will royally screw up results!
		// Setting the link-grammar max linkages to a low number,
		// e.g. 4, when there are a dozen or more parses, will cause a
		// RANDOM 4 sentences to be returned out of the dozen, instead
		// of the top 4.  We almost surely do NOT want that! Or rather,
		// I'm assuming no one actually wants that; we want the top 4!!
		// So, setting the link-grammar max linkages iss a huge mistake!
		// if (config.getMaxLinkages() > -1)
		//	LinkGrammar.setMaxLinkages(config.getMaxLinkages());
	}

	/**
	 * Assuming <code>LinkGrammar.parse</code> has already been called,
	 * construct a full <code>ParseResult</code> given the passed in
	 * configuration. For example, no more that
	 * <code>config.getMaxLinkages</code> are returned, etc.
	 *
	 * @param config
	 * @return
	 */
	public static ParseResult getAsParseResult(LGConfig config)
	{
		LinkGrammar.makeLinkage(0); // need to call at least once, otherwise it crashes
		ParseResult parseResult = new ParseResult();
		parseResult.setParserVersion(LinkGrammar.getVersion());
		parseResult.setDictVersion(LinkGrammar.getDictVersion());
		parseResult.numSkippedWords = LinkGrammar.getNumSkippedWords();

		int maxLinkages = Math.min(config.getMaxLinkages(), LinkGrammar.getNumLinkages());
		for (int li = 0; li < maxLinkages; li++)
		{
			LinkGrammar.makeLinkage(li);
			Linkage linkage = new Linkage();
			linkage.setDisjunctCost(LinkGrammar.getLinkageDisjunctCost());
			linkage.setLinkCost(LinkGrammar.getLinkageLinkCost());
			linkage.setLinkedWordCount(LinkGrammar.getNumWords());
			linkage.setNumViolations(LinkGrammar.getLinkageNumViolations());
			String [] disjuncts = new String[LinkGrammar.getNumWords()];
			String [] words = new String[LinkGrammar.getNumWords()];
			for (int i = 0; i < words.length; i++)
			{
				disjuncts[i] = LinkGrammar.getLinkageDisjunct(i);
				words[i] = LinkGrammar.getLinkageWord(i);
			}
			linkage.setWords(words);
			linkage.setDisjuncts(disjuncts);
			int numLinks = LinkGrammar.getNumLinks();
			for (int i = 0; i < numLinks; i++)
			{
				Link link = new Link();
				link.setLabel(LinkGrammar.getLinkLabel(i));
				link.setLeft(LinkGrammar.getLinkLWord(i));
				link.setRight(LinkGrammar.getLinkRWord(i));
				link.setLeftLabel(LinkGrammar.getLinkLLabel(i));
				link.setRightLabel(LinkGrammar.getLinkRLabel(i));
				linkage.getLinks().add(link);
			}
			if (config.isStoreConstituentString())
				linkage.setConstituentString(LinkGrammar.getConstituentString());
			if (config.isStoreDiagramString())
				linkage.setDiagramString(LinkGrammar.getLinkString());
			parseResult.linkages.add(linkage);
		}
		return parseResult;
	}

	/**
	 * Construct a JSON formatted result for a parse which yielded 0 linkages.
	 */
	public static String getEmptyJSONResult(LGConfig config)
	{
		StringBuffer buf = new StringBuffer();
		buf.append("\"numSkippedWords\":0,");
		buf.append("\"linkages\":[],");
		buf.append("\"version\":\"" + LinkGrammar.getVersion() + "\",");
		buf.append("\"dictVersion\":\"" + LinkGrammar.getDictVersion() + "\"}");
		return buf.toString();
	}

	/**
	 * Format the current parsing result as a JSON string. This method
	 * assume that <code>LinkGrammar.parse</code> has been called before.
	 */
	public static String getAsJSONFormat(LGConfig config)
	{
		LinkGrammar.makeLinkage(0); // need to call at least once, otherwise it crashes
		int numWords = LinkGrammar.getNumWords();
		int maxLinkages = Math.min(config.getMaxLinkages(), LinkGrammar.getNumLinkages());
		StringBuffer buf = new StringBuffer();
		buf.append("{\"numSkippedWords\":" + LinkGrammar.getNumSkippedWords());
		buf.append(",\"linkages\":[");
		for (int li = 0; li < maxLinkages; li++)
		{
			LinkGrammar.makeLinkage(li);
			buf.append("{\"words\":[");
			for (int i = 0; i < numWords; i++)
			{
				buf.append(JSONUtils.jsonString(LinkGrammar.getLinkageWord(i)));
				if (i + 1 < numWords)
					buf.append(",");
			}
			buf.append("], \"disjuncts\":[");
			for (int i = 0; i < numWords; i++)
			{
				buf.append(JSONUtils.jsonString(LinkGrammar.getLinkageDisjunct(i)));
				if (i + 1 < numWords)
					buf.append(",");
			}
			buf.append("], \"disjunctCost\":");
			buf.append(Double.toString(LinkGrammar.getLinkageDisjunctCost()));
			buf.append(", \"linkageCost\":");
			buf.append(Double.toString(LinkGrammar.getLinkageLinkCost()));
			buf.append(", \"numViolations\":");
			buf.append(Integer.toString(LinkGrammar.getLinkageNumViolations()));
			if (config.isStoreConstituentString())
			{
				buf.append(", \"constituentString\":");
				buf.append(JSONUtils.jsonString(LinkGrammar.getConstituentString()));
			}
			if (config.isStoreDiagramString())
			{
				buf.append(", \"diagramString\":");
				buf.append(JSONUtils.jsonString(LinkGrammar.getLinkString()));
			}
			buf.append(", \"links\":[");
			int numLinks = LinkGrammar.getNumLinks();
			for (int i = 0; i < numLinks; i++)
			{
				buf.append("{\"label\":" + JSONUtils.jsonString(LinkGrammar.getLinkLabel(i)) + ",");
				buf.append("\"left\":" + LinkGrammar.getLinkLWord(i) + ",");
				buf.append("\"right\":" + LinkGrammar.getLinkRWord(i) + ",");
				buf.append("\"leftLabel\":" + JSONUtils.jsonString(LinkGrammar.getLinkLLabel(i)) + ",");
				buf.append("\"rightLabel\":" + JSONUtils.jsonString(LinkGrammar.getLinkRLabel(i)) + "}");
				if (i + 1 < numLinks)
					buf.append(",");
			}
			buf.append("]");
			buf.append("}");
			if (li < maxLinkages - 1)
				buf.append(",");
		}
		buf.append("],\"version\":\"" + LinkGrammar.getVersion() + "\"");
		buf.append(",\"dictVersion\":\"" + LinkGrammar.getDictVersion() + "\"");
		buf.append("}");
		return buf.toString();
	}

	/**
	 * A stub method for now for implementing a compact binary format
	 * for parse results.
	 *
	 * @param config
	 * @return
	 */
	public static byte [] getAsBinary(LGConfig config)
	{
		int size = 0;
		byte [] buf = new byte[1024];
		// TODO ..... grow buf as needed
		byte [] result = new byte[size];
		System.arraycopy(buf, 0, result, 0, size);
		return result;
	}

	private static void handleClient(Socket clientSocket)
	{
		init();
		Reader in = null;
		PrintWriter out = null;
		JSONUtils msgreader = new JSONUtils();
		try
		{
			trace("Connection accepted from : " + clientSocket.getInetAddress());
			in = new InputStreamReader(clientSocket.getInputStream());
			Map<String, String> msg = msgreader.readMsg(in);
			if (verbose)
				trace("Received msg '" + msg + "' from " + clientSocket.getInetAddress());
			String json = "{}";
			if ("version".equals(msg.get("get"))) // special case msg 'get:version'
				json = "{\"version\":\"" + LinkGrammar.getVersion() + "\"}";
			else
			{
				LGConfig config = new LGConfig();
				config.setMaxCost(JSONUtils.getDouble("maxCost", msg, config.getMaxCost()));
				config.setMaxLinkages(JSONUtils.getInt("maxLinkages", msg, config.getMaxLinkages()));
				config.setMaxParseSeconds(JSONUtils.getInt("maxParseSeconds", msg, config.getMaxParseSeconds()));
				config.setStoreConstituentString(
					JSONUtils.getBool("storeConstituentString", msg, config.isStoreConstituentString()));
				config.setStoreDiagramString(
					JSONUtils.getBool("storeDiagramString", msg, config.isStoreDiagramString()));
				configure(config);
				String text = msg.get("text");
				if (text != null && text.trim().length() > 0)
				{
					LinkGrammar.parse(text);
					if (LinkGrammar.getNumLinkages() > 0)
						json = getAsJSONFormat(config);
					else
						json = getEmptyJSONResult(config);
				}
				else
					json = getEmptyJSONResult(config);
			}
			out = new PrintWriter(clientSocket.getOutputStream(), true);
			out.print(json.length() + 1);
			out.print('\n');
			out.print(json);
			out.print('\n');
			out.flush();
			trace("Response written to " + clientSocket.getInetAddress() + ", closing client connection...");
		}
		catch (Throwable t)
		{
			t.printStackTrace(System.err);
		}
		finally
		{
			if (out != null) try { out.close(); } catch (Throwable t) { }
			if (in != null) try { in.close(); } catch (Throwable t) { }
			if (clientSocket != null) try { clientSocket.close(); } catch (Throwable t) { }
		}
	}

	/**
	 * <p>
	 * Parse a piece of text with the given configuration and return
	 * the <code>ParseResult</code>.
	 * </p>
	 *
	 * @param config The configuration to be used. If this is the first
	 * time the <code>parse</code> method is called within the current
	 * thread, the dictionary location (if not <code>null</code>) of
	 * this parameter will be used to initialize the parser. Otherwise
	 * the dictionary location is ignored.
	 *
	 * @param text The text to parse, normally a single sentence.
	 *
	 * @return The <code>ParseResult</code>. Note that <code>null</code>
	 * is never returned. If parsing failed, there will be 0 linkages in
	 * the result.
	 */
	public static ParseResult parse(LGConfig config, String text)
	{
		init();
		configure(config);
		LinkGrammar.parse(text);
		return getAsParseResult(config);
	}

	public static void main(String [] argv)
	{
		int threads = 1;
		int port = 0;
		String dictionaryPath = null;
		String language = null;
		try
		{
			int argIdx = 0;
			if (argv[argIdx].equals("-verbose")) { verbose = true; argIdx++; }
			if (argv[argIdx].equals("-threads")) { threads = Integer.parseInt(argv[++argIdx]); argIdx++; }
			port = Integer.parseInt(argv[argIdx++]);
			if (argv.length > argIdx)
				language = argv[argIdx++];
			if (argv.length > argIdx)
				dictionaryPath = argv[argIdx++];
		}
		catch (Throwable ex)
		{
			if (argv.length > 0)
				ex.printStackTrace(System.err);
			System.out.println("Usage: java org.linkgrammar.LGService [-verbose] [-threads n] port [language] [dictPath]");
			System.out.println("Start a link-grammar parse server on tcp/ip port.  The server returns");
			System.out.println("JSON-formated parse results.  Socket input should be a single sentence");
			System.out.println("to parse, proceeded by the identifier \"text:\".\n");
			System.out.println("  'port'      The TCP port the service should listen to.");
			System.out.println("  -verbose    Generate verbose output.");
			System.out.println("  -threads    Number of concurrent threads/clients allowed (default 1).");
			System.out.println("  'language'  Language abbreviation (en, ru, de, lt or fr).");
			System.out.println("  'dictPath'  Full path to the Link-Grammar dictionaries.");
			System.exit(-1);
		}

		if (dictionaryPath != null)
		{
			File f = new File(dictionaryPath);
			if (!f.exists())
			{ System.err.println("Dictionary path " + dictionaryPath + " not found."); System.exit(-1); }
			else if (!f.isDirectory())
			{ System.err.println("Dictionary path " + dictionaryPath + " not a directory."); System.exit(-1); }
		}

		System.out.println("Starting Link Grammar Server at port " + port +
				", with " + threads + " available processing threads and " +
				((dictionaryPath == null) ? " with default dictionary location." :
					"with dictionary location '" + dictionaryPath + "'."));
		ThreadPoolExecutor threadPool = new ThreadPoolExecutor(threads,
		                                        threads,
		                                        Long.MAX_VALUE,
		                                        TimeUnit.SECONDS,
		                                        new LinkedBlockingQueue<Runnable>());
		try
		{
			if (language != null)
				LinkGrammar.setLanguage(language);
			if (dictionaryPath != null)
				LinkGrammar.setDictionariesPath(dictionaryPath);
			ServerSocket serverSocket = new ServerSocket(port);
			while (true)
			{
				trace("Waiting for client connections...");
				final Socket clientSocket = serverSocket.accept();
				threadPool.submit(new Runnable()
				{
					public void run()
					{
						// We must catch in here, and not outside the
						// thread pool, because the submit method will
						// silently swallow the exception. There is a
						// CERT advisory for this feature/bug:
						// http://www.securecoding.cert.org/confluence/display/java/TPS03-J.+Ensure+that+tasks+executing+in+a+thread+pool+do+not+fail+silently
						try
						{
							handleClient(clientSocket);
						}
						catch (Throwable t)
						{
							t.printStackTrace(System.err);
							System.exit(-1);
						}
					}
				});
			}
		}
		catch (Throwable t)
		{
			t.printStackTrace(System.err);
			System.exit(-1);
		}
	}
}
