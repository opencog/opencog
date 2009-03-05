//vfrom should dereference to a pair<var_idx,tree::node>
vector<DTree> model;
vector<packed_t> data;

//intitial call:
vector<pair<int,DTree::iterator> > roots
(make_pair_iterator
 (make_counting_iterator(0),
  make_tranform_iterator(model.begin(),bind(&Dtree::begin,_1))),
 make_pair_iterator
 (make_counting_iterator(model.size()),
  make_tranform_iterator(model.end(),bind(&Dtree::begin,_1))));
rec_build_model(roots.begin(),roots.end(),0,N,maxDepth);

void rec_build_model(It tgt_from,It tgt_to,int lower,int upper,int maxDepth) {
  if (maxDepth==0)
    return;

  for (It tgt=tgt_from;tgt!=tgt_to;++tgt) {
    gain_t max_gain=0;
    int best_idx;
    for (int src_idx=0;src_idx!=get_idx(*tgt);++src_idx) {
      if (split_allowed(src_idx,*tgt)) {
	gain_t g=compute_gain(data[src_idx],data[get_idx(*tgt)],lower,upper);
	if (g>max_gain) {
	  max_gain=g;
	  best_idx=src_idx;
	}
      }
    }
    if (max_gain>0)
      best_by_src[best_idx].push_back(*tgt);
  }

  //only works for nway splits
  for (split_map::const_iterator src=best_by_src.begin();
       src!=best_by_src.end();++src) {
    std::vector<int> pivots;
    partition(src->first,lower,upper,std::back_inserter(pivots));
      
    //split all of the target nodes on source
    for_each(src->second.begin(),src->second.end(),bind(split,_1,src->first));

    //recurse pivots.size()+1 times, try the new targets
    for_each_range([rlower,rupper],pivots)
      rec_build_model(ith children of the target nodes,
		      rlower,rupper,maxDepth-1);
  }	   target!=max_scoring_by_source[source].end();++target) {
	
	
    

	  


  for (It target=begin();target!=vto;++target) {
    for (int sourceIdx=0;sourceIdx!=get_idx(*target);++sourceIdx) {
      score[source,target]=split_allowed(sourceIdx,*target) ?
	compute_score(data[sourceIdx],data[get_idx(target)],lower,upper) : -1;
    }
  }

  hash_map<int,vector<It> > max_scoring_by_source;
  for (It target=vfrom;target!=vto;++target) {
    max_score=negative_infinity;
    for (It source=vfrom;source!=target;++source) {
      if (score[source,target]>max_score)
	max_score=score[source,target];
    }
    max_scoring_by_source[source].push_back(target);
  }

  for (It source=vfrom;source!=vto;++source) {
    if (!max_scoring_by_source[source].empty()) {
      pivots[source.cardinality()]=partition_on(source,lower,upper);
      
      //split all of the trees - vfrom must be a tree location
      
      for_each_range([rlower,rupper],pivots) {
	rec_build_model(vfrom,
	

      for (It target=max_scoring_by_source[source].begin();
	   target!=max_scoring_by_source[source].end();++target) {
	
	
    

      
