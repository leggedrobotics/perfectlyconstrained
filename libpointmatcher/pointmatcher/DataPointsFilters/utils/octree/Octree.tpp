// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <iterator>
#include <future>
#include <ciso646>

#include "OctreeLookupTable.h"

template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(): parent{nullptr},	depth{0}
{
	for(size_t i=0; i< nbCells; ++i)
		cells[i] = nullptr;
}

template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(const Octree_<T,dim>& o): 
	bb{o.bb.center, o.bb.radius}, depth{o.depth}
{
	if (!o.parent) // Root case
		parent = nullptr;	
		
	if(o.isLeaf()) // Leaf case
	{
		// Nullify childs.
		for(size_t i=0; i<nbCells; ++i)
			cells[i]= nullptr;
		// Copy data.
		data.insert(data.end(), o.data.begin(), o.data.end());
	}
	else // Node case
	{
		//Create each child recursively
		for(size_t i=0; i<nbCells;++i)
		{
			// Create a copy of the child, calling this copy constructor until we reach a leaf.
			cells[i] = new Octree_<T,dim>(*(o.cells[i]));	

			// Assign parent to the child copy.
			cells[i]->parent = this;
		}	
	}
}
template<typename T, std::size_t dim>
Octree_<T,dim>::Octree_(Octree_<T,dim>&& o): 
	parent{nullptr}, bb{o.bb.center, o.bb.radius}, depth{o.depth}
{
	// Only allow move of root node
	assert(o.isRoot());

	if(o.isLeaf()) // Leaf case
	{
		// Move data
		data.insert(data.end(), 
			std::make_move_iterator(o.data.begin()), 
			std::make_move_iterator(o.data.end()));
	}
	// Move childs.
	for(size_t i=0; i<nbCells; ++i)
	{
		// Direct assignment of child.
		cells[i] = o.cells[i];

		// Nullify ptrs of argument octree children.
		o.cells[i] = nullptr;
	}
}

template<typename T, std::size_t dim>
Octree_<T,dim>::~Octree_()
{
	// delete recursively childs
	if(!isLeaf())
		for(size_t i=0; i<nbCells; ++i)
			delete cells[i];
}

template<typename T, std::size_t dim>	
Octree_<T,dim>& Octree_<T,dim>::operator=(const Octree_<T,dim>& o)
{
	if (!o.parent) 
		parent = nullptr; // Root case
	
	// Set the depth of the octree to the same value.
	depth = o.depth;
	
	if(o.isLeaf()) // Leaf case
	{
		// Nullify children.
		for(size_t i=0; i<nbCells; ++i)
			cells[i]= nullptr;

  		// Copy data
  		data.insert(data.end(), o.data.begin(), o.data.end());
	}
	else // Node case
	{
		// Create each child recursively
  		for(size_t i=0; i<nbCells; ++i)
  		{
			  // Create a copy of the child, calling this copy constructor until we reach a leaf.
  			cells[i] = new Octree_<T,dim>(*(o.cells[i]));	
			
			// Assign parent to the child copy.
  			cells[i]->parent = this;
  		}	
	}
	return *this;
}

template<typename T, std::size_t dim>	
Octree_<T,dim>& Octree_<T,dim>::operator=(Octree_<T,dim>&& o)
{
	//only allow move of root node
	assert(o.isRoot());
	
	parent = nullptr;
	bb.center = o.bb.center;
	bb.radius = o.bb.radius;
	
	depth = o.depth;
	
	if(o.isLeaf()) //Leaf case
	{
		//Copy data
		data.insert(data.end(), 
			std::make_move_iterator(o.data.begin()), 
			std::make_move_iterator(o.data.end()));
	}
	
	// Copy children through pointers.
	for(size_t i=0; i<nbCells; ++i)
	{
		// Direct assignment of child.
		cells[i] = o.cells[i];

		// Nullify ptrs of argument octree children.
		o.cells[i] = nullptr;
	}
	
	return *this;
}

template<typename T, std::size_t dim>
bool Octree_<T,dim>::isLeaf() const
{
	return (cells[0]==nullptr);
}
template<typename T, std::size_t dim>
bool Octree_<T,dim>::isRoot() const
{
	return (parent==nullptr);
}
template<typename T, std::size_t dim>
bool Octree_<T,dim>::isEmpty() const
{
	return (data.size() == 0);
}
template<typename T, std::size_t dim>
size_t Octree_<T,dim>::idx(const Point& pt) const
{
	size_t id = 0;

	for(size_t i=0; i<dim; ++i)
	{
		const bool pointCoordinateLargerThanBoundingBoxCenter{pt(i) > bb.center(i)};
		id |= (pointCoordinateLargerThanBoundingBoxCenter << i);
	}

	return id;
}

template<typename T, std::size_t dim>
size_t Octree_<T,dim>::idx(const DP& pts, const Data d) const
{
	return idx(pts.features.col(d).head(dim));
}
template<typename T, std::size_t dim>
const Octree_<T,dim>* Octree_<T,dim>::getCells() const
{
	return cells;
}
template<typename T, std::size_t dim>
size_t Octree_<T,dim>::getDepth() const
{
	return depth;
}
template<typename T, std::size_t dim>
T Octree_<T,dim>::getRadius() const
{
	return bb.radius;
}
template<typename T, std::size_t dim>
typename Octree_<T,dim>::Point Octree_<T,dim>::getCenter() const
{
	return bb.center;
}
template<typename T, std::size_t dim>
typename Octree_<T,dim>::DataContainer* Octree_<T,dim>::getData()
{
	return &data;
}
template<typename T, std::size_t dim>
Octree_<T,dim>* Octree_<T,dim>::operator[](size_t idx)
{
	assert(idx<nbCells);
	return cells[idx];
}

template<typename T, std::size_t dim>
typename Octree_<T,dim>::DataContainer Octree_<T,dim>::toData(const DP& /*pts*/, const std::vector<Id>& ids)
{
	return DataContainer{ids.begin(), ids.end()};
}

// Build tree from DataPoints with a specified number of points by node
template<typename T, std::size_t dim>
bool Octree_<T,dim>::build(const DP& pts, size_t maxDataByNode, T maxSizeByNode, bool parallelBuild, bool centerAtOrigin)
{
	typedef typename PM::Vector Vector;
	
	//Build bounding box
	BoundingBox box;
	
	// Compute vector of minimum values, row-wise.
	Vector minValues{pts.features.rowwise().minCoeff()};
	Vector maxValues{pts.features.rowwise().maxCoeff()};
	
	// Compute minimum and maximum values from the whole point cloud.
	Point min{minValues.head(dim)};
	Point max{maxValues.head(dim)};
	
	// Compute parameters of bounding box.
	Point radii{max - min};

	if(centerAtOrigin) {
		box.center = Point::Zero();
	} else {
		box.center = min + radii * 0.5;
	}

	// Set radius to be the max of all radii.
	const double x{radii.maxCoeff() * 0.5};
	box.radius = std::pow(2, std::ceil(std::log(x)/std::log(2)));
	
	// Fill the data vector with indices of points in the point cloud.
	const size_t numPoints{pts.getNbPoints()};

	// TODO(ynava) This loop can be optimized by filling values with std::fil?
	std::vector<Id> indexes;
	indexes.reserve(numPoints);
	for(size_t i=0; i<numPoints; ++i)
		indexes.emplace_back(Id(i));
	
	DataContainer datas = toData(pts, indexes);
	
	// Recursive call to build.
	return this->build(pts, std::move(datas), std::move(box), maxDataByNode, maxSizeByNode, parallelBuild);
}

template<typename T, std::size_t dim>
bool Octree_<T,dim>::build(const DP& pts, DataContainer&& datas, BoundingBox && bb, 
	size_t maxDataByNode, T maxSizeByNode, bool parallelBuild)
{
	// Assign bounding box parameters from parent's.
	this->bb.center = bb.center;
	this->bb.radius = bb.radius;

	// Validate stop condition.
	if((bb.radius * 2.0 <= maxSizeByNode) or (datas.size() <= maxDataByNode))
	{		
		// Insert data
		data.insert(data.end(), 
			std::make_move_iterator(datas.begin()), std::make_move_iterator(datas.end()));
		return isLeaf();
	}
	
	/* Potential for optimization */
	/* In this block the data vector (size N) is split into multiple sections. The implementation reserves space
	for M vectors of size N, appends the data, and then shrinks the vectors to their final size */
	/* An alternative approach would be to pre-compute which points would go into each cell, and based on that allocate
	space on each of the M vectors */
	// Split data into cells.
	const std::size_t nbData{datas.size()};

	// Reserve space in vector for the data points.
	DataContainer sDatas[nbCells];
	for(size_t i=0; i<nbCells; ++i)
		sDatas[i].reserve(nbData);
	
	// Copy data points into this node's data vector.
	for(auto&& d : datas)
		(sDatas[idx(pts, d)]).emplace_back(d);
	
	// Shrink data container to fit content.
	for(size_t i=0; i<nbCells; ++i)
		sDatas[i].shrink_to_fit();
	/* Potential for optimization */
	
	// Compute new bounding boxes for children.
	BoundingBox boxes[nbCells];
	const T halfRadius{this->bb.radius * 0.5f};
	for(size_t i=0; i<nbCells; ++i)
	{
		const Point offset{OctreeLookupTable<T,dim>::offsetTable[i] * this->bb.radius};
		boxes[i].radius = halfRadius;
		boxes[i].center = this->bb.center + offset;
	}
	
	// For each child build bottom levels of the octree recursively.
	std::vector<std::future<void>> futures;
	for(size_t i=0; i<nbCells; ++i)
	{		
		auto compute = [maxDataByNode, maxSizeByNode, i, &pts, &sDatas, &boxes, this](){
				this->cells[i] = new Octree_<T,dim>();
				//Assign depth
				this->cells[i]->depth = this->depth + 1;
				//Assign parent
				this->cells[i]->parent = this;
				//next call is not parallelizable
				this->cells[i]->build(pts, std::move(sDatas[i]), std::move(boxes[i]), maxDataByNode, maxSizeByNode, false);	
			};
		
		if(parallelBuild)
			futures.push_back( std::async( std::launch::async, compute ));
		else
			compute();
	}

	// If parallel build is enabled, wait for the octree to be ready.
	for(auto& f : futures) f.get();

	return !isLeaf();
}

//------------------------------------------------------------------------------
template<typename T, std::size_t dim>
template<typename Callback>
bool Octree_<T,dim>::visit(Callback& cb)
{
	// Call the callback for this node (if the callback returns false, then
	// stop traversing.
	if (!cb(*this)) return false;

	// If I'm a node, recursively traverse my children
	if (!isLeaf())
		for (size_t i=0; i<nbCells; ++i)
			if (!cells[i]->visit(cb)) return false;

	return true;
}
