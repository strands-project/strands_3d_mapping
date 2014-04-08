#include "KeyPointSet.h"
#include <algorithm>
#include <vector>

KeyPointSet::KeyPointSet(){stabilety_threshold 	= 0;};

KeyPointSet::~KeyPointSet(){
	for(unsigned int i = 0; i < valid_key_points.size(); i++)	{delete valid_key_points.at(i);}
	for(unsigned int i = 0; i < invalid_key_points.size(); i++)	{delete invalid_key_points.at(i);}
};

//bool comparison(KeyPoint * a, KeyPoint * b) { return (a->stabilety > b->stabilety); }

void KeyPointSet::sortKeyPoints(){
	//sort (valid_key_points.begin()	, valid_key_points.end()	, comparison);
	//sort (invalid_key_points.begin(), invalid_key_points.end()	, comparison);
}
