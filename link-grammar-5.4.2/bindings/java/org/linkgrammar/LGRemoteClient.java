package org.linkgrammar;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.List;
import java.util.Map;

/**
 * <p>
 * A client of the {@link LGService} when it is run in server mode. The main
 * method to call is {@link parse} which produces a {@link ParseResult}. At
 * a minimum, the <code>hostname</code> and <code>port</code> properties
 * must be set beforehand. Configuration settings of the remote parser itself
 * are specified with the <code>config</code> property.
 * </p>
 *
 * <p>
 * This client is persistent in its attempts to perform a parse. By default
 * it will keep trying to connect to a server indefinitely (unless the host is
 * unknown). To change that,
 * set the <code>connectRetryCount</code> property. Also, if call to the server
 * fails for whatever reason, it will retry at least once. The increase the number
 * of retries, set the <code>parseRetryCount</code> property.
 * </p>
 *
 * @author Borislav Iordanov
 *
 */
public class LGRemoteClient
{
    private LGConfig config = new LGConfig();
    private String parserVersion;
    private String hostname = "localhost";
    private int port = 9000;
    private int parseRetryCount = 2;
    private int connectRetryCount = Integer.MAX_VALUE;
    private long connectRetryWait = 1000l;

    @SuppressWarnings({"unchecked"})
    private ParseResult jsonToParseResult(String json)
    {
        JSONReader reader = new JSONReader();
        Map top = (Map)reader.read(json);
        ParseResult result = new ParseResult();
        result.setParserVersion((String)top.get("version"));
        result.setNumSkippedWords(((Number)top.get("numSkippedWords")).intValue());
        for (Map x : (List<Map>)top.get("linkages"))
        {
            Linkage linkage = new Linkage();
            linkage.setDisjunctCost(((Number)x.get("disjunctCost")).doubleValue());
            linkage.setLinkCost(((Number)x.get("linkageCost")).doubleValue());
            linkage.setNumViolations(((Number)x.get("numViolations")).intValue());
            linkage.setWords(((List<String>)(x.get("words"))).toArray(new String[0]));
            linkage.setDisjuncts(((List<String>)(x.get("disjuncts"))).toArray(new String[0]));
            linkage.setLinkedWordCount(linkage.getWords().length); // TODO?? is this right?
            for (Map y : (List<Map>)x.get("links"))
            {
                Link link = new Link();
                link.setLabel((String)y.get("label"));
                link.setLeftLabel((String)y.get("leftLabel"));
                link.setRightLabel((String)y.get("rightLabel"));
                link.setLeft(((Number)y.get("left")).intValue());
                link.setRight(((Number)y.get("right")).intValue());
                linkage.getLinks().add(link);
            }
            if (config.isStoreConstituentString())
                linkage.setConstituentString((String)x.get("constituentString"));
            result.getLinkages().add(linkage);
        }
        return result;
    }

    private String makeLGRequest(String text)
    {
        if (config != null)
            return "storeConstituentString:" + config.isStoreConstituentString() + "\0" +
                   "maxCost:" + config.getMaxCost() + "\0" +
                   "maxLinkages:" + config.getMaxLinkages() + "\0" +
                   "maxParseSeconds:" + config.getMaxParseSeconds() + "\0" +
                   "text:" + text + "\0";
        else
            return "text:" + text + "\0";
    }

    private String readResponse(BufferedReader in, int size) throws IOException
    {
        char [] buf = new char[size];
        for (int count = 0; count < size; )
            count += in.read(buf, count, size-count);
        return new String(buf, 0, size);
    }

    private String callParser(String request) throws InterruptedException, IOException
    {
        if (hostname == null || hostname.length() == 0 || port <= 1024)
            throw new RuntimeException("No hostname for remote parser or invalid port number < 1024");

        //
        // Connect:
        //
        Socket socket = null;

        for (int i = 0; i < connectRetryCount && socket == null; i++)
        {
            try
            {
                socket = new Socket(hostname, port);
            }
            catch (UnknownHostException ex)
            {
                throw new RuntimeException("Host '" + hostname + "' not found.");
            }
            catch (IOException ex)
            {
                // ignore, retry...
            }
            if (socket == null)
                Thread.sleep(connectRetryWait);
        }

        if (socket == null)
            throw new RuntimeException("Failed to connect to " + hostname + ":" + port);

        //
        // Call parser:
        //
        PrintWriter out = null;
        BufferedReader in = null;
        try
        {
            out = new PrintWriter(socket.getOutputStream(), true);
            in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            out.print(request);
            out.print('\n');
            out.flush();
            String size = in.readLine();
            if (size == null)
                throw new RuntimeException("Parser returned no response.");
            return readResponse(in, Integer.parseInt(size));
        }
        finally
        {
            if (out != null) try { out.close(); } catch (Throwable t) { }
            if (in != null) try { in.close(); } catch (Throwable t) { }
            try { socket.close(); } catch (Throwable t) { }
        }
    }

    /**
     * <p>Return the link grammar version. A call to the server is made to
     * obtain the version which is henceforth cached.</p>
     */
    @SuppressWarnings("unchecked")
    public String getVersion()
    {
        if (parserVersion == null)
        {
            try
            {
                String json = callParser("get:version\0");
                JSONReader reader = new JSONReader();
                Map top = (Map)reader.read(json);
                parserVersion = (String)top.get("version");
            }
            catch (IOException ex)
            {
                parserVersion = "unavailable";
            }
            catch (InterruptedException ex)
            {
                throw new RuntimeException("Thread interrupted.", ex);
            }
        }
        return parserVersion;
    }

    public ParseResult parse(String sentence) throws InterruptedException
    {
        String parserResponse = null;
        for (int i = 0; i < parseRetryCount && parserResponse == null; i++)
            try { parserResponse = callParser(makeLGRequest(sentence)); }
            catch (IOException ex)
            {
                if (i == 0) // Trace exception only on the first failure.
                {
                    System.err.println("Link grammar called failed on '" +
                                       sentence + "'" + ", will retry " +
                                       Integer.toString(parseRetryCount - 1) +
                                       " more time.");
                    ex.printStackTrace();
                }
            }
        if (parserResponse == null)
            return null;
        ParseResult parseResult = jsonToParseResult(parserResponse);
        parseResult.setText(sentence);
        return parseResult;
    }

    public LGConfig getConfig()
    {
        return config;
    }

    public void setConfig(LGConfig config)
    {
        this.config = config;
    }

    public String getHostname()
    {
        return hostname;
    }

    public void setHostname(String hostname)
    {
        this.hostname = hostname;
    }

    public int getPort()
    {
        return port;
    }

    public void setPort(int port)
    {
        this.port = port;
    }

    public int getParseRetryCount()
    {
        return parseRetryCount;
    }

    public void setParseRetryCount(int parseRetryCount)
    {
        this.parseRetryCount = parseRetryCount;
    }

    public int getConnectRetryCount()
    {
        return connectRetryCount;
    }

    public void setConnectRetryCount(int connectRetryCount)
    {
        this.connectRetryCount = connectRetryCount;
    }

    public long getConnectRetryWait()
    {
        return connectRetryWait;
    }

    public void setConnectRetryWait(long connectRetryWait)
    {
        this.connectRetryWait = connectRetryWait;
    }
}
