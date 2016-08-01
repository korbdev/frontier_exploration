/*
 * FibNode.h
 *
 *  Created on: 03.07.2016
 *      Author: rkorb
 */

#ifndef FIBNODE_H_
#define FIBNODE_H_

#include "FHeap.h"

template <class T>
class FHeap;

template <class T>
class FibNode{
	friend class FHeap<T>;
private:
	double key;
	T content;
	int rank;
	bool mark;
	FibNode *parent, *child, *left,*right;
public:
	FibNode(double key, T content):key(key),content(content),rank(0),mark(false),parent(0L),child(0L),left(this),right(this){}
	FibNode():key(0.0),content(0),rank(0),mark(false),parent(0L),child(0L),left(this),right(this){}
	double getKey(){return key;}
	T getContent(){return content;}
};



#endif /* FIBNODE_H_ */
