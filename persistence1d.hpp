/*! \file persistence1d.hpp
    Actual code.
*/

#ifndef PERSISTENCE_H
#define PERSISTENCE_H

#include <assert.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

#define NO_COLOR -1
#define RESIZE_FACTOR 20
#define MATLAB_INDEX_FACTOR 1

namespace p1d 
{

/** Used to sort data according to its absolute value and refer to its original index in the Data vector.

        A collection of TIdxData is sorted according to its data value (if values are equal, according
        to indices). The index allows access back to the vertex in the Data vector.
*/
struct TIdxData
{
    TIdxData():idx(-1), data(0){}

    bool operator<(const TIdxData& other) const
    {
        if (data < other.data) return true;
        if (data > other.data) return false;
        return (idx < other.idx);
    }

    ///The index of the vertex within the data vector.
    int idx;

    ///Vertex data value from the original Data vector sent as an argument to runPersistence.
    float data;
};


/*! Defines a component within the data domain. 
        A component is created at a local minimum - a vertex whose value is smaller than both of its neighboring
        vertices' values.
*/
struct TComponent
{
    ///A component is defined by the indices of its edges.
    ///Both variables hold the respective indices of the vertices in Data vector.
    ///All vertices between them are considered to belong to this component.
    int leftEdgeIndex;
    int rightEdgeIndex;

    ///The index of the local minimum within the component as longs as its alive.
    int minIndex;

    ///The value of the Data[MinIndex].
    float minValue; //redundant, but makes life easier

    ///Set to true when a component is created. Once components are merged,
    ///the destroyed component Alive value is set to false.
    ///Used to verify correctness of algorithm.
    bool alive;
};


/** A pair of matched local minimum and local maximum
        that define a component above a certain persistence threshold.
        The persistence value is their (absolute) data difference.
*/
struct TPairedExtrema
{
    ///Index of local minimum, as per Data vector.
    int minIndex;

    ///Index of local maximum, as per Data vector.
    int maxIndex;

    ///The persistence of the two extrema.
    ///Data[MaxIndex] - Data[MinIndex]
    ///Guaranteed to be >= 0.
    float persistence;

    bool operator<(const TPairedExtrema& other) const
    {
        if (persistence < other.persistence) return true;
        if (persistence > other.persistence) return false;
        return (minIndex < other.minIndex);
    }
};



/*! Finds extrema and their persistence in one-dimensional data.

  Local minima and local maxima are extracted, paired,
  and sorted according to their persistence.
  The global minimum is extracted as well.

  We assume a connected one-dimensional domain.
  Think of "data on a line", or a function f(x) over some domain xmin <= x <= xmax.
*/
class Persistence1D
{
public:
    Persistence1D()
    {
    }

    ~Persistence1D()
    {
    }

    /*!
      Call this function with a vector of one dimensional data to find extrema features in the data.
      The function runs once for, results can be retrieved with different persistent thresholds without
      further data processing.

      Input data vector is assumed to be of legal size and legal values.

      Use printResults, getPairedExtrema or getExtremaIndices to get results of the function.

      @param[in] inputData Vector of data to find features on, ordered according to its axis.
    */
    bool runPersistence(const std::vector<float>& inputData)
    {
        data = inputData;
        init();

        //If a user runs this on an empty vector, then they should not get the results of the previous run.
        if (data.empty()) return false;

        createIndexValueVector();
        watershed();
        sortPairedExtrema();
#ifdef _DEBUG
        verifyAliveComponents();
#endif
        return true;
    }



    /*!
                Prints the contents of the TPairedExtrema vector.
                If called directly with a TPairedExtrema vector, the global minimum is not printed.

                @param[in] pairs	Vector of pairs to be printed.
        */
    void printPairs(const std::vector<TPairedExtrema>& pairs) const
    {
        for (std::vector<TPairedExtrema>::const_iterator it = pairs.begin();
             it != pairs.end(); it++)
        {
            std::cout	<< "Persistence: " << (*it).persistence
                        << " minimum index: " << (*it).minIndex
                        << " maximum index: " << (*it).maxIndex
                        << std::endl;
        }
    }

    /*!
       Prints the global minimum and all paired extrema whose persistence is greater or equal to threshold.
       By default, all pairs are printed. Supports Matlab indexing.

       @param[in] threshold		Threshold value for pair persistence.
       @param[in] matlabIndexing	Use Matlab indexing for printing.
     */
    void printResults(const float threshold = 0.0, const bool matlabIndexing = false) const
    {
        if (threshold < 0)
        {
            std::cout << "Error. Threshold value must be greater than or equal to 0" << std::endl;
        }
        if (threshold==0 && !matlabIndexing)
        {
            printPairs(pairedExtrema);
        }
        else
        {
            std::vector<TPairedExtrema> pairs;
            getPairedExtrema(pairs, threshold, matlabIndexing);
            printPairs(pairs);
        }

        std::cout << "Global minimum value: " << getGlobalMinimumValue()
                  << " index: " << getGlobalMinimumIndex(matlabIndexing)
                  << std::endl;
    }


    /*!
       Use this method to get the results of runPersistence.
       Returned pairs are sorted according to persistence, from least to most persistent.

       @param[out]	pairs			Destination vector for PairedExtrema
       @param[in]	threshold		Minimal persistence value of returned features. All PairedExtrema
                                                               with persistence equal to or above this value will be returned.
                                                               If left to default, all PairedMaxima will be returned.

       @param[in] matlabIndexing	Set this to true to change all indices of features to Matlab's 1-indexing.
     */
    bool getPairedExtrema(std::vector<TPairedExtrema> & pairs, const float threshold = 0, const bool matlabIndexing = false) const
    {
        //make sure the user does not use previous results that do not match the data
        pairs.clear();

        if (pairedExtrema.empty() || threshold < 0.0) return false;

        std::vector<TPairedExtrema>::const_iterator lower_bound = filterByPersistence(threshold);

        if (lower_bound == pairedExtrema.end()) return false;

        pairs = std::vector<TPairedExtrema>(lower_bound, pairedExtrema.end());

        if (matlabIndexing) //match matlab indices by adding one
        {
            for (std::vector<TPairedExtrema>::iterator p = pairs.begin(); p != pairs.end(); p++)
            {
                (*p).minIndex += MATLAB_INDEX_FACTOR;
                (*p).maxIndex += MATLAB_INDEX_FACTOR;
            }
        }
        return true;
    }

    /*!
      Use this method to get two vectors with all indices of PairedExterma.
      Returns false if no paired features were found.
      Returned vectors have the same length.
      Overwrites any data contained in min, max vectors.

      @param[out] min				Vector of indices of paired local minima.
      @param[out]	max				Vector of indices of paired local maxima.
      @param[in]	threshold		Return only indices for pairs whose persistence is greater than or equal to threshold.
      @param[in]	matlabIndexing	Set this to true to change all indices to match Matlab's 1-indexing.
    */
    bool getExtremaIndices(std::vector<int> & min, std::vector<int> & max, const float threshold = 0, const bool matlabIndexing = false) const
    {
        //before doing anything, make sure the user does not use old results
        min.clear();
        max.clear();

        if (pairedExtrema.empty() || threshold < 0.0) return false;

        min.reserve(pairedExtrema.size());
        max.reserve(pairedExtrema.size());

        int matlabIndexFactor = 0;
        if (matlabIndexing) matlabIndexFactor = MATLAB_INDEX_FACTOR;

        std::vector<TPairedExtrema>::const_iterator lower_bound = filterByPersistence(threshold);

        for (std::vector<TPairedExtrema>::const_iterator p = lower_bound; p != pairedExtrema.end(); p++)
        {
            min.push_back((*p).minIndex + matlabIndexFactor);
            max.push_back((*p).maxIndex + matlabIndexFactor);
        }
        return true;
    }
    /*!
      Returns the index of the global minimum.
      The global minimum does not get paired and is not returned
      via getPairedExtrema and getExtremaIndices.
    */
    int getGlobalMinimumIndex(const bool matlabIndexing = false) const
    {
        if (components.empty()) return -1;

        assert(components.front().alive);
        if (matlabIndexing)
        {
            return components.front().minIndex + 1;
        }

        return components.front().minIndex;
    }

    /*!
      Returns the value of the global minimum.
      The global minimum does not get paired and is not returned
      via getPairedExtrema and getExtremaIndices.
    */
    float getGlobalMinimumValue() const
    {
        if (components.empty()) return 0;

        assert(components.front().alive);
        return components.front().minValue;
    }
    /*!
       Runs basic sanity checks on results of runPersistence:
       - Number of unique minima = number of unique maxima - 1 (Morse property)
       - All returned indices are unique (no index is returned as two extrema)
       - Global minimum is within domain indices or at default value
       - Global minimum is not returned as any other extrema.
       - Global minimum is not paired.

       Returns true if run results pass these sanity checks.
    */
    bool verifyResults()
    {
        bool flag = true;
        std::vector<int> min, max;
        std::vector<int> combinedIndices;

        getExtremaIndices(min, max);

        int globalMinIdx = getGlobalMinimumIndex();

        std::sort(min.begin(), min.end());
        std::sort(max.begin(), max.end());
        combinedIndices.reserve(min.size() + max.size());
        std::set_union(min.begin(), min.end(), max.begin(), max.end(), std::inserter(combinedIndices, combinedIndices.begin()));

        //check the combined unique indices are equal to size of min and max
        if (combinedIndices.size() != (min.size() + max.size()) ||
                std::binary_search(combinedIndices.begin(), combinedIndices.end(), globalMinIdx) == true)
        {
            flag = false;
        }

        if ((globalMinIdx > (int)data.size()-1) || (globalMinIdx < -1)) flag = false;
        if (globalMinIdx == -1 && min.size() != 0) flag = false;

        std::vector<int>::iterator minUniqueEnd = std::unique(min.begin(), min.end());
        std::vector<int>::iterator maxUniqueEnd = std::unique(max.begin(), max.end());

        if (minUniqueEnd != min.end() ||
                maxUniqueEnd != max.end() ||
                (minUniqueEnd - min.begin()) != (maxUniqueEnd - max.begin()))
        {
            flag = false;
        }

        return flag;
    }

protected:
    /*!
      Contain a copy of the original input data.
     */
    std::vector<float> data;


    /*!
      Contains a copy the value and index pairs of Data, sorted according to the data values.
     */
    std::vector<TIdxData> sortedData;


    /*!
      Contains the Component assignment for each vertex in Data.
      Only edges of destroyed components are updated to the new component color.
      The Component values in this vector are invalid at the end of the algorithm.
     */
    std::vector<int> colors;		//need to init to empty


    /*!
      A vector of components.
      The component index within the vector is used as its colors in the watershed function.
     */
    std::vector<TComponent> components;


    /*!
      A vector of paired extrema features - always a minimum and a maximum.
     */
    std::vector<TPairedExtrema> pairedExtrema;


    unsigned int totalComponents;	//keeps track of component vector size and newest component "color"
    bool aliveComponentsVerified;	//Index of global minimum in Data vector. This minimum is never paired.


    /*!
      Merges two components by doing the following:

      - Destroys component with smaller hub (sets Alive=false).
      - Updates surviving component's edges to span the destroyed component's region.
      - Updates the destroyed component's edge vertex colors to the survivor's color in colors[].

      @param[in] firstIdx,secondIdx	Indices of components to be merged. Their order does not matter.
    */
    void mergeComponents(const int firstIdx, const int secondIdx)
    {
        int survivorIdx, destroyedIdx;
        //survivor - component whose hub is bigger
        if (components[firstIdx].minValue < components[secondIdx].minValue)
        {
            survivorIdx = firstIdx;
            destroyedIdx = secondIdx;
        }
        else if (components[firstIdx].minValue > components[secondIdx].minValue)
        {
            survivorIdx = secondIdx;
            destroyedIdx = firstIdx;
        }
        else if (firstIdx < secondIdx) // Both components min values are equal, destroy component on the right
            // This is done to fit with the left-to-right total ordering of the values
        {
            survivorIdx = firstIdx;
            destroyedIdx = secondIdx;
        }
        else
        {
            survivorIdx = secondIdx;
            destroyedIdx = firstIdx;
        }

        //survivor and destroyed are decided, now destroy!
        components[destroyedIdx].alive = false;

        //Update the color of the edges of the destroyed component to the color of the surviving component.
        colors[components[destroyedIdx].rightEdgeIndex] = survivorIdx;
        colors[components[destroyedIdx].leftEdgeIndex] = survivorIdx;

        //Update the relevant edge index of surviving component, such that it contains the destroyed component's region.
        if (components[survivorIdx].minIndex > components[destroyedIdx].minIndex) //destroyed index to the left of survivor, update left edge
        {
            components[survivorIdx].leftEdgeIndex = components[destroyedIdx].leftEdgeIndex;
        }
        else
        {
            components[survivorIdx].rightEdgeIndex = components[destroyedIdx].rightEdgeIndex;
        }
    }

    /*!
      Creates a new PairedExtrema from the two indices, and adds it to PairedFeatures.

      @param[in] firstIdx, secondIdx Indices of vertices to be paired. Order does not matter.
     */
    void createPairedExtrema(const int firstIdx, const int secondIdx)
    {
        TPairedExtrema pair;

        //There might be a potential bug here, todo (we're checking data, not sorted data)
        //example case: 1 1 1 1 1 1 -5 might remove if after else
        if (data[firstIdx] > data[secondIdx])
        {
            pair.maxIndex = firstIdx;
            pair.minIndex = secondIdx;
        }
        else if (data[secondIdx] > data[firstIdx])
        {
            pair.maxIndex = secondIdx;
            pair.minIndex = firstIdx;
        }
        //both values are equal, choose the left one as the min
        else if (firstIdx < secondIdx)
        {
            pair.minIndex = firstIdx;
            pair.maxIndex = secondIdx;
        }
        else
        {
            pair.minIndex = secondIdx;
            pair.maxIndex = firstIdx;
        }

        pair.persistence = data[pair.maxIndex] - data[pair.minIndex];

#ifdef _DEBUG
        assert(pair.Persistence >= 0);
#endif
        if (pairedExtrema.capacity() == pairedExtrema.size())
        {
            pairedExtrema.reserve(pairedExtrema.size() * 2 + 1);
        }

        pairedExtrema.push_back(pair);
    }


    // Changing the alignment of the next Doxygen comment block breaks its formatting.

    /*!
      Creates a new component at a local minimum.

      Neighboring vertices are assumed to have no color.
      - Adds a new component to the components vector,
      - Initializes its edges and minimum index to minIdx.
      - Updates colors[minIdx] to the component's color.

      @param[in]	minIdx Index of a local minimum.
     */
    void createComponent(const int minIdx)
    {
        TComponent comp;
        comp.alive = true;
        comp.leftEdgeIndex = minIdx;
        comp.rightEdgeIndex = minIdx;
        comp.minIndex = minIdx;
        comp.minValue = data[minIdx];

        //place at the end of component vector and get the current size
        if (components.capacity() <= totalComponents)
        {
            components.reserve(2 * totalComponents + 1);
        }

        components.push_back(comp);
        colors[minIdx] = totalComponents;
        totalComponents++;
    }


    /*!
      Extends the component's region by one vertex:

      - Updates the matching component's edge to dataIdx..
      - updates colors[dataIdx] to the component's color.

      @param[in]	componentIdx	Index of component (the value of a neighboring vertex in colors[]).
      @param[in] 	dataIdx			Index of vertex which the component is extended to.
    */
    void extendComponent(const int componentIdx, const int dataIdx)
    {
#ifdef _DEUBG
        assert(components[componentIdx].Alive == true)
        #endif

                //extend to the left
                if (dataIdx + 1 == components[componentIdx].leftEdgeIndex)
        {
            components[componentIdx].leftEdgeIndex = dataIdx;
        }
        //extend to the right
        else if (dataIdx - 1 == components[componentIdx].rightEdgeIndex)
        {
            components[componentIdx].rightEdgeIndex = dataIdx;
        }
        else
        {
#ifdef _DEUBG
            std::string errorMessage = "extendComponent: index mismatch. Data index: ";
            errorMessage += std::to_string((long long)dataIdx);
            throw (errorMessage);
#endif 
        }

        colors[dataIdx] = componentIdx;
    }


    /*!
      Initializes main data structures used in class:
      - Sets colors[] to NO_COLOR
      - Reserves memory for components and pairedExtrema

      Note: sortedData is should be created before, separately, using createIndexValueVector()
    */
    void init()
    {
        sortedData.clear();
        sortedData.reserve(data.size());

        colors.clear();
        colors.resize(data.size());
        std::fill(colors.begin(), colors.end(), NO_COLOR);

        int vectorSize = (int)(data.size()/RESIZE_FACTOR) + 1; //starting reserved size >= 1 at least

        components.clear();
        components.reserve(vectorSize);

        pairedExtrema.clear();
        pairedExtrema.reserve(vectorSize);

        totalComponents = 0;
        aliveComponentsVerified = false;
    }


    /*!
      Creates sortedData vector.
      Assumes Data is already set.
     */
    void createIndexValueVector()
    {
        if (data.size()==0) return;

        for (std::vector<float>::size_type i = 0; i != data.size(); i++)
        {
            TIdxData dataidxpair;

            //this is going to make problems
            dataidxpair.data = data[i];
            dataidxpair.idx = (int)i;

            sortedData.push_back(dataidxpair);
        }

        std::sort(sortedData.begin(), sortedData.end());
    }


    /*!
      Main algorithm - all of the work happen here.

      Use only after calling createIndexValueVector and init functions.

      Iterates over each vertex in the graph according to their ordered values:
      - Creates a segment for each local minima
      - Extends a segment is data has only one neighboring component
      - Merges segments and creates new pairedExtrema when a vertex has two neighboring components.
    */
    void watershed()
    {
        if (sortedData.size()==1)
        {
            createComponent(0);
            return;
        }

        for (std::vector<TIdxData>::iterator p = sortedData.begin(); p != sortedData.end(); p++)
        {
            int i = (*p).idx;

            //left most vertex - no left neighbor
            //two options - either local minimum, or extend component
            if (i==0)
            {
                if (colors[i+1] == NO_COLOR)
                {
                    createComponent(i);
                }
                else
                {
                    extendComponent(colors[i+1], i);  //in this case, local max as well
                }

                continue;
            }
            else if (i == colors.size()-1) //right most vertex - look only to the left
            {
                if (colors[i-1] == NO_COLOR)
                {
                    createComponent(i);
                }
                else
                {
                    extendComponent(colors[i-1], i);
                }
                continue;
            }

            //look left and right
            if (colors[i-1] == NO_COLOR && colors[i+1] == NO_COLOR) //local minimum - create new component
            {
                createComponent(i);
            }
            else if (colors[i-1] != NO_COLOR && colors[i+1] == NO_COLOR) //single neighbor on the left - extnd
            {
                extendComponent(colors[i-1], i);
            }
            else if (colors[i-1] == NO_COLOR && colors[i+1] != NO_COLOR) //single component on the right - extend
            {
                extendComponent(colors[i+1], i);
            }
            else if (colors[i-1] != NO_COLOR && colors[i+1] != NO_COLOR) //local maximum - merge components
            {
                int leftComp, rightComp;

                leftComp = colors[i-1];
                rightComp = colors[i+1];

                //choose component with smaller hub destroyed component
                if (components[rightComp].minValue < components[leftComp].minValue) //left component has smaller hub
                {
                    createPairedExtrema(components[leftComp].minIndex, i);
                }
                else	//either right component has smaller hub, or hubs are equal - destroy right component.
                {
                    createPairedExtrema(components[rightComp].minIndex, i);
                }

                mergeComponents(leftComp, rightComp);
                colors[i] = colors[i-1]; //color should be correct at both sides at this point
            }
        }
    }


    /*!
      Sorts the pairedExtrema list according to the persistence of the features.
      Orders features with equal persistence according the the index of their minima.
     */
    void sortPairedExtrema()
    {
        std::sort(pairedExtrema.begin(), pairedExtrema.end());
    }


    /*!
      Returns an iterator to the first element in pairedExtrema whose persistence is bigger or equal to threshold.
      If threshold is set to 0, returns an iterator to the first object in pairedExtrema.

      @param[in]	threshold	Minimum persistence of features to be returned.
     */
    std::vector<TPairedExtrema>::const_iterator filterByPersistence(const float threshold = 0) const
    {
        if (threshold == 0 || threshold < 0) return pairedExtrema.begin();

        TPairedExtrema searchPair;
        searchPair.persistence = threshold;
        searchPair.maxIndex = 0;
        searchPair.minIndex = 0;
        return(lower_bound(pairedExtrema.begin(), pairedExtrema.end(), searchPair));
    }
    /*!
      Runs at the end of runPersistence, after watershed.
      Algorithm results should be as followed:
      - All but one components should not be Alive.
      - The Alive component contains the global minimum.
      - The Alive component should be the first component in the Component vector
    */
    bool verifyAliveComponents()
    {
        //verify that the Alive component is component #0 (contains global minimum by definition)
        if ((*components.begin()).alive != true)
        {

#ifndef _DEBUG 
            return false;
#endif 
#ifdef _DEBUG
            throw "Error. Component 0 is not Alive, assumed to contain global minimum";
#endif
        }

        for (std::vector<TComponent>::const_iterator it = components.begin()+1; it != components.end(); it++)
        {
            if ((*it).alive == true)
            {

#ifndef _DEBUG 
                return false;
#endif 
#ifdef _DEBUG
                throw "Error. Found more than one alive component";
#endif
            }
        }

        return true;
    }
};
}
#endif
