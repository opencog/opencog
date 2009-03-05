#ifndef _COMBO_MESSAGE_H
#define _COMBO_MESSAGE_H

#include <LADSUtil/exceptions.h>

#include "ComboReduct/combo/type_tree_def.h"
#include "ComboReduct/combo/operator_base.h"

#define COMBO_MESSAGE_PREFIX "message:"

namespace combo {

  //message is essentially a string but is coded as a different type
  //as definite_object because it semantically denotes something else
  class message {
  private:
    std::string _content;
  public:
    message(std::string m) {
      _content = m;
    }

    std::string getContent() const {
      return _content;
    }

    bool operator==(message m) const {
      return _content==m.getContent();
    }
    bool operator!=(message m) const {
      return _content!=m.getContent();
    }
    bool operator<(message m) const {
      return _content<m.getContent();
    }
    bool operator<=(message m) const {
      return _content<=m.getContent();
    }
    bool operator>(message m) const {
      return _content>m.getContent();
    }
    bool operator>=(message m) const {
      return _content>=m.getContent();
    }
    
    static std::string prefix() {
      return COMBO_MESSAGE_PREFIX;
    }
  };

  typedef std::set<message> message_set;
  typedef message_set::iterator message_set_it;
  typedef message_set::const_iterator message_set_const_it;

}//~namespace combo

std::ostream& operator<<(std::ostream&,const combo::message&);

#endif

