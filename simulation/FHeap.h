/*
 * FibonacciHeap.h
 *
 *  Created on: 03.07.2016
 *      Author: rkorb
 */
//

#ifndef FIBONACCIHEAP_H_
#define FIBONACCIHEAP_H_

#include "FibNode.h"
#include "math.h"
#include <vector>
#include <iostream>

template <class T>
class FHeap{
private:
	FibNode<T> *min;
	int size;
public:
	FHeap():min(0L), size(0){}
	FibNode<T>* insert(double key, T content);
	static FibNode<T>* mergeLists(FibNode<T>* a, FibNode<T>* b);
	FibNode<T>* link(FibNode<T>* a, FibNode<T>* b);
	void cut(FibNode<T>* node);
	void cascadingCut(FibNode<T>* node);
	FibNode<T>* remove(FibNode<T>* node);
	FibNode<T>* getMin();
	int getSize();
	bool deleteMin(FibNode<T> *result);
	bool isEmpty();
	void consolidate();
	int maxRank();
	void print();
};

template <class T>
FibNode<T>* FHeap<T>::getMin(){
	return min;
}

template <class T>
int FHeap<T>::getSize(){
	return size;
}

template <class T>
bool FHeap<T>::isEmpty(){
	return size == 0 ? true:false;
}

template <class T>
void FHeap<T>::print(){
	FibNode<T> *node = min;
	for(int i = 0; i < size; i++){
		std::cout << node->key << " Content " << node->content << std::endl;
		node = node->right;
	}
}

template <class T>
FibNode<T>* FHeap<T>::insert(double key, T content){
	FibNode<T>* node = new FibNode<T>(key, content);
	min = mergeLists(min, node);
	size++;
	return node;
}

template <class T>
void FHeap<T>::cut(FibNode<T>* node){
	if(node->parent != 0L){
		node->left->right = node->left;
		node->right->left = node->right;

		node->parent->rank--;
		node->parent = 0L;
		size++;
		mergeLists(min, node);
		node->mark = false;
	}
}

template <class T>
void FHeap<T>::cascadingCut(FibNode<T>* node){
	if(node->parent){
		if(node->mark){
			cut(node);
			cascadingCut(node->parent);
		} else {
			node->mark = true;
		}
	}
}

template <class T>
FibNode<T>* FHeap<T>::remove(FibNode<T>* node){
	size--;
	FibNode<T>* n = node->child;
	for(int i = 0; i<node->rank; i++){
		n->parent = 0L;
		FibNode<T>* temp = n;
		n = n->left;
		mergeLists(min, temp);
		size++;
	}
	node->left->right = node->right;
	node->right->left = node->left;
	if(node == min){
		min = node->left;
	}
	return node;
}

template <class T>
FibNode<T>* FHeap<T>::mergeLists(FibNode<T>* a, FibNode<T>* b){
	if(a == 0L && b == 0L){
		return 0L;
	}
	if(a == 0L){
		b->left = b;
		b->right = b;
		return b;
	}
	if(b == 0L){
		a->left = a;
		a->right = a;
		return a;
	}

	b->left = a;
	b->right = a->right;
	b->right->left = b;
	a->right = b;

	if(b->key < a->key){
		return b;
	}
	else{
		return a;
	}
}

template <class T>
FibNode<T>* FHeap<T>::link(FibNode<T>* a, FibNode<T>* b){
	if(a->rank != b->rank){
		return 0L;
	}
	FibNode<T> *parent, *child;
	if(a->key < b->key){
		parent = a;
		child = b;
	}
	else if(a == min){
		parent = a;
		child = b;
	}
	else if(b == min){
		parent = b;
		child = a;
	}
	else{
		parent = b;
		child = a;
	}

	child->left->right = child->right;
	child->right->left = child->left;

	child->parent = parent;
	parent->rank++;
	size--;
	child->mark = false;
	parent->child = mergeLists(parent->child, child);
	return parent;
}
template <class T>
int FHeap<T>::maxRank(){
	//double test = 2*log2(size);
	return 15;
	return 2*(log(size)/log(1.618f));
}

template <class T>
void FHeap<T>::consolidate(){
	int rankSize = maxRank()+1;
	std::vector<FibNode<T>*> rankVec(rankSize);

	FibNode<T>* node = min;
	FibNode<T>* next;
	int initial_size = size;
	for(int i = 0; i < initial_size; i++){
		next = node->left;
		//std::cout << "Node " << node->key << ", " << node->content << " Rank " << node->rank << " MaxRank " << rankSize << std::endl;
		while(rankVec[node->rank] != 0){//&& rankVec[node->rank] != node){
			FibNode<T>* node2 = rankVec[node->rank];
			node = link(node, node2);
			rankVec[node->rank-1] = 0L;
		}
		rankVec[node->rank] = node;
		node = next;
	}
}

template <class T>
bool FHeap<T>::deleteMin(FibNode<T> *result){
	if(size > 0){
		FibNode<T>*backup = this->remove(min);
		FibNode<T>* node = min;
		for(int i = 0; i < size; i++){
			node = node->left;
			if(node->key < min->key){
				min = node;
			}
		}

		if(size > 1){
			this->consolidate();
		}

		result->key = backup->key;
		result->content = backup->content;
		delete backup;
		return true;
	}
	else{
		return false;
	}
}

#endif /* FIBONACCIHEAP_H_ */
