#ifndef _SOCKETS_Debug_H
#define _SOCKETS_Debug_H

#include "sockets-config.h"
#include <string>
#include "Utility.h"
#include <map>


#ifdef SOCKETS_NAMESPACE
namespace SOCKETS_NAMESPACE {
#endif


class Debug
{
static	const char *colors[];
public:
	class endl {
	public:
		endl() {}
		virtual ~endl() {}
	};

public:
	Debug() {}
	Debug(const std::string& x) : m_id(0), m_text(x) {
		fprintf(stderr, "%s", colors[Utility::ThreadID() % 16 + 1]);
		for (int i = 0; i < m_level[Utility::ThreadID()]; i++)
			fprintf(stderr, "  ");
		fprintf(stderr, "%s%s\n", x.c_str(), colors[0]);
		m_level[Utility::ThreadID()]++;
	}
	Debug(int id, const std::string& x) : m_id(id), m_text(x) {
		fprintf(stderr, "%s", colors[Utility::ThreadID() % 16 + 1]);
		for (int i = 0; i < m_level[Utility::ThreadID()]; i++)
			fprintf(stderr, "  ");
		fprintf(stderr, "%d> %s%s\n", m_id, x.c_str(), colors[0]);
		m_level[Utility::ThreadID()]++;
	}
	~Debug() {
		if (!m_text.empty())
		{
			if (m_level[Utility::ThreadID()])
				m_level[Utility::ThreadID()]--;
			fprintf(stderr, "%s", colors[Utility::ThreadID() % 16 + 1]);
			for (int i = 0; i < m_level[Utility::ThreadID()]; i++)
				fprintf(stderr, "  ");
			if (m_id)
				fprintf(stderr, "%d> /%s%s\n", m_id, m_text.c_str(), colors[0]);
			else
				fprintf(stderr, "/%s%s\n", m_text.c_str(), colors[0]);
			fflush(stderr);
		}
	}
static	void Print(const char *format, ...);

	Debug& operator<<(const std::string& );
	Debug& operator<<(long);
	Debug& operator<<(endl);

private:
	int m_id;
	std::string m_text;
static	std::map<unsigned long, int> m_level;
	std::string m_line;
};


#ifdef SOCKETS_NAMESPACE
} // namespace SOCKETS_NAMESPACE {
#endif

#endif // _SOCKETS_Debug_H

