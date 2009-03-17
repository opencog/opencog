#include <Sockets/HttpClientSocket.h>

class RouterHttpPostSocket : public HttpClientSocket {
    public:
        /* client constructor,
         * \param url_in = 'http://host:port/resource' 
         */
        RouterHttpPostSocket(ISocketHandler&,const std::string& url_in);
        ~RouterHttpPostSocket();

        /** Add body to post. */
        void SetBody(const std::string& body);

        /** connect to host:port derived from url in constructor */
        void Open();

        /** http post client implemented in OnConnect */
        void OnConnect();

    private: 

        std::string body;

        //std::string m_host;
        //port_t m_port;

};
