namespace combo
{
namespace id {
enum ann_id {
ann, ann_node, ann_input //ann additions
};
}
typedef id::ann_id ann_id;
class ann_type {
public:
    explicit ann_type(int i,ann_id _id) : idx(i),id(_id) {
    }
    int idx;
    ann_id id;

    void negate() {
        idx = -idx;
    }
    bool operator<(ann_type rhs) const {
        return idx < rhs.idx;
    } 
    bool operator==(ann_type rhs) const {
        return idx == rhs.idx;
    }
    bool operator!=(ann_type rhs) const {
        return idx != rhs.idx;
    }
};
}
