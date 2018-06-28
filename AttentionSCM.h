#include <opencog/atoms/base/Link.h>
#include <opencog/attentionbank/AttentionBank.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/atoms/proto/LinkValue.h>
#include <opencog/attentionbank/AFImplicator.h>


namespace opencog {

class AttentionSCM
{
	protected:
		static void* init_in_guile(void*);
		static void init_in_module(void*);
		void init(void);
	public:
		AttentionSCM(void);
		~AttentionSCM();

		AttentionValuePtr get_av(const Handle&);
		Handle set_av(const Handle&, const AttentionValuePtr&);
		Handle inc_vlti(const Handle&);
		Handle dec_vlti(const Handle&);

		Handle update_af(int);
		int af_size(void);
		int set_af_size(int);
		Handle stimulate (const Handle&, double);

		Handle af_bindlink(const Handle&);
};
}
