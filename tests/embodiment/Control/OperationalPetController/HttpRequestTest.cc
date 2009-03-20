/**
 * HttpRequestTest.cc
 *
 * Just some integration tests with Proxy, using HTTP requests.
 *
 * Author: Welter
 */

#include <Sockets/StdoutLog.h>
#include <Sockets/SocketHandler.h>
#include <Sockets/HttpPutSocket.h>
#include <Sockets/HttpPostSocket.h>
#include <Sockets/HTTPSocket.h>

class MyHTTPSocket : public HTTPSocket {

    public: 	
        MyHTTPSocket(ISocketHandler& handler) : HTTPSocket(handler) {}

        /** Callback executes when first line has been received.
         *                 GetMethod, GetUrl/GetUri, and GetHttpVersion are valid when this callback is executed. */
        void OnFirst() {
            printf("OnFirst() called!\n");
        }

        /** For each header line this callback is executed.
        *                 \param key Http header name
        *                                 \param value Http header value */
        void OnHeader(const std::string& key,const std::string& value) {
            printf("OnHeader() called!\n");
	}

        /** Callback fires when all http headers have been received. */
        void OnHeaderComplete() {
            printf("OnHeaderComplete() called!\n");
	}

        /** Chunk of http body data recevied. */
        void OnData(const char * data,size_t size) {
            printf("OnData() called! Size = %d, data = %s\n", size, data);
	}

};

class HttpRequestTest {  
	std::string actionPlanMsg; 
public:
    HttpRequestTest(){
        actionPlanMsg = std::string("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n") +
            "<pet:action-plan\n" +
                "xmlns:pet=\" http://proxy.esheepco.com/brain\"\n" +
                "xmlns:xsi=\" http://www.w3.org/2001/XMLSchema-instance\"\n" +
                "xsi:schemaLocation=\" http://proxy.esheepco.com/brain BrainProxyAxon.xsd\"\n" + 
                "id=\"5dfe52f9-7344-4ffe-99b7-493f428d5b34\"\n" + 
                "pet-id=\"78\">\n" +
              "<parallel>\n" +
                "<action name=\"run\" sequence=\"1\">\n" +
                  "<param name=\"to\" type=\"vector\">\n" +
                    "<vector x=\"1\" y=\"2\" z=\"3\"/>\n" +
                  "</param>\n" +
                  "<param name=\"speed\" type=\"int\" value=\"10\"/>\n" +
                "</action>\n" +
                "<action name=\"wagTail\" sequence=\"2\"/>\n" +
              "</parallel>\n" +
              "<action name=\"tapDance\" sequence=\"3\">\n" +
                "<param name=\"duration\" type=\"int\" value=\"5\"/>\n" +
              "</action>\n" +
              "<action name=\"walk\" sequence=\"4\">\n" +
                "<param name=\"to\" type=\"vector\">\n" +
                  "<vector x=\"10\" y=\"20\" z=\"30\"/>\n" +
                "</param>\n" +
                "<param name=\"speed\" type=\"int\" value=\"5\"/>\n" +
              "</action>\n" +
            "</pet:action-plan>\n" +
            "";
    }
    
    ~HttpRequestTest(){
    }
    
    void testHttpRequest() {
        StdoutLog log;
        SocketHandler h(&log);
#if 0	
        MyHTTPSocket sock( h);
	sock.Open("localhost", 8211);
	//sock.Open("200.188.191.139", 8211);
        h.Add(&sock);
        h.Select(1,0);
        while (h.GetCount())
        {
           h.Select(1,0);
        }

	std::string url = "http://localhost:8211/petproxy/pet/1/action/1";
	//std::string url = "http://www.uai.com.br";
       	sock.SetUrl(url);
	std::string requestMethod = "POST";
	//std::string requestMethod = "GET";
	sock.SetMethod(requestMethod);
	sock.SendRequest();

        std::string planId = "1";
        std::string petId = "1";
        std::string actionPlanXml = 
            std::string("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n") +
                        "<pet:action-plan\n" +
                        "  xmlns:pet=\"http://proxy.esheepco.com/brain\"\n" +
                        "  xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n" +
                        "  xsi:schemaLocation=\"http://proxy.esheepco.com/brain BrainProxyAxon.xsd\"" +
                        "  id=\"" + planId + "\"" +
                        "  pet-id=\"" + petId + "\">\n" +
                        "    <parallel>\n" +
                        "        <action name=\"run\" sequence=\"1\">\n" +
                        "            <param name=\"to\" type=\"vector\">\n" +
                        "                <vector x=\"1\" y=\"2\" z=\"3\"/>\n" +
                        "            </param>\n" +
                        "            <param name=\"speed\" type=\"int\" value=\"10\"/>\n" +
                        "        </action>\n" +
                        "        <action name=\"wagTail\" sequence=\"2\"/>\n" +
                        "    </parallel>\n" +
                        "    <action name=\"tapDance\" sequence=\"3\">\n" +
                        "        <param name=\"duration\" type=\"int\" value=\"5\"/>\n" +
                        "    </action>\n" +
                        "    <action name=\"walk\" sequence=\"4\">\n" +
                        "        <param name=\"to\" type=\"vector\">\n" +
                        "            <vector x=\"10\" y=\"20\" z=\"30\"/>\n" +
                        "        </param>\n" +
                        "        <param name=\"speed\" type=\"int\" value=\"5\"/>\n" +
                        "    </action>\n" +
                        "</pet:action-plan>\n" +
                        "";
	sock.Send(actionPlanXml);

        while (h.GetCount())
        {
           h.Select(1,0);
        }

        printf("Status of operation: %s %s\n", sock.GetStatus().c_str(), sock.GetStatusText().c_str());
#else
        //HttpPostSocket sock( h, "http://localhost:8211/petproxy/pet/1/action/1" ); 
        //HttpPostSocket sock( h, "http://localhost:8211/petproxy/pet/78/action/5dfe52f9-7344-4ffe-99b7-493f428d5b34" ); 
        HttpPostSocket sock( h, "http://localhost:8211/petproxy/pet/1/action" ); // IS THIS REALLY VALID?  
        sock.Open();
        h.Add(&sock);
        //sock.AddFile("actionPlan", "ActionPlanExample.xml", "text/plain");
        sock.AddField("actionPlan", actionPlanMsg);
        h.Select(1,0);
        while (h.GetCount())
        {
            h.Select(1,0);
        }
        
        printf("Status of operation: %s %s\n", sock.GetStatus().c_str(), sock.GetStatusText().c_str());
#endif
    }

};

int main(int argc, char* argv[]) {

    HttpRequestTest test;
    test.testHttpRequest();
    return 0;
}
