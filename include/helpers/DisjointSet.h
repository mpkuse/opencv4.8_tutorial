#pragma once
#include <iostream>
#include <map>
#include <memory>

namespace datastruct
{

    namespace mp
    {

        /**
        @brief  A disjoint set forest is a fairly standard data structure used to represent the partition of
                a set of elements into disjoint sets in such a way that common operations such as merging two
                sets together are computationally efficient.

        This implementation uses the well-known union-by-rank and path compression optimizations, which together
        yield an amortised complexity for key operations of O(a(n)), where a is the (extremely slow-growing)
        inverse of the Ackermann function.

        The implementation also allows clients to attach arbitrary data to each element, which can be useful for
        some algorithms.

        @tparam T   The type of data to attach to each element (arbitrary)
        */
        template <typename T>
        class DisjointSetForest
        {
            //#################### NESTED CLASSES ####################
        private:
            struct Element
            {
                T m_value;
                int m_parent;
                int m_rank;

                Element(const T &value, int parent)
                    : m_value(value), m_parent(parent), m_rank(0)
                {
                }
            };

            //#################### PRIVATE VARIABLES ####################
        private:
            mutable std::map<int, Element> m_elements;
            int m_setCount;

            //#################### CONSTRUCTORS ####################
        public:
            /**
            @brief  Constructs an empty disjoint set forest.
            */
            DisjointSetForest()
                : m_setCount(0)
            {
            }

            /**
            @brief  Constructs a disjoint set forest from an initial set of elements and their associated values.

            @param[in]  initialElements     A map from the initial elements to their associated values
            */
            explicit DisjointSetForest(const std::map<int, T> &initialElements)
                : m_setCount(0)
            {
                add_elements(initialElements);
            }

            //#################### PUBLIC METHODS ####################
        public:
            /**
            @brief  Adds a single element x (and its associated value) to the disjoint set forest.

            @param[in]  x       The index of the element
            @param[in]  value   The value to initially associate with the element
            @pre
                -   x must not already be in the disjoint set forest
            */
            void add_element(int x, const T &value = T())
            {
                m_elements.insert(std::make_pair(x, Element(value, x)));
                ++m_setCount;
            }

            int append_element(const T &value)
            {
                int g = element_count();
                add_element(g, value);
                return g;
            }

            /**
            @brief  Adds multiple elements (and their associated values) to the disjoint set forest.

            @param[in]  elements    A map from the elements to add to their associated values
            @pre
                -   None of the elements to be added must already be in the disjoint set forest
            */
            void add_elements(const std::map<int, T> &elements)
            {
                for (typename std::map<int, T>::const_iterator it = elements.begin(), iend = elements.end(); it != iend; ++it)
                {
                    m_elements.insert(std::make_pair(it->first, Element(it->second, it->first)));
                }
                m_setCount += elements.size();
            }

            /**
            @brief  Returns the number of elements in the disjoint set forest.

            @return As described
            */
            int element_count() const
            {
                return static_cast<int>(m_elements.size());
            }

            /**
            @brief  Finds the index of the root element of the tree containing x in the disjoint set forest.

            @param[in]  x   The element whose set to determine
            @pre
                -   x must be an element in the disjoint set forest
            @throw Exception
                -   If the precondition is violated
            @return As described
            */
            int find_set(int x) const
            {
                Element &element = get_element(x);
                int &parent = element.m_parent;
                if (parent != x)
                {
                    parent = find_set(parent);
                }
                return parent;
            }

            std::vector<int> get_all_root_slow()
            {
                // loop over keys of map
                std::vector<int> list_of_roots;
                for (const auto &entry : m_elements)
                {
                    int key = entry.first;
                    if (find_set(key) == key)
                    {
                        list_of_roots.push_back(key);
                    }
                }
                return list_of_roots;
            }

            std::vector<int> get_all_x_in_set(int x)
            {
                int root_x = find_set(x);
                std::vector<int> in_set;
                for (const auto &entry : m_elements)
                {
                    int key = entry.first;
                    if (find_set(key) == root_x)
                    {
                        in_set.push_back(key);
                    }
                }
                return in_set;
            }

            std::vector<T> get_all_items_in_set(int x)
            {
                int root_x = find_set(x);
                std::vector<T> in_set;
                for (const auto &entry : m_elements)
                {
                    int key = entry.first;
                    if (find_set(key) == root_x)
                    {
                        in_set.push_back(entry.second.m_value);
                    }
                }
                return in_set;
            }

            int exists(const T &vx)
            {
                for (const auto &entry : m_elements)
                {
                    if (entry.second.m_value == vx)
                    {
                        return entry.first;
                    }
                }
                return -1;
            }

            /**
            @brief  Returns the current number of disjoint sets in the forest (i.e. the current number of trees).

            @return As described
            */
            int set_count() const
            {
                return m_setCount;
            }

            /**
            @brief  Merges the disjoint sets containing elements x and y.

            If both elements are already in the same disjoint set, this is a no-op.

            @param[in]  x   The first element
            @param[in]  y   The second element
            @pre
                -   Both x and y must be elements in the disjoint set forest
            @throw Exception
                -   If the precondition is violated
            */
            void union_sets(int x, int y)
            {
                int setX = find_set(x);
                int setY = find_set(y);
                if (setX != setY)
                    link(setX, setY);
            }

            /**
            @brief  Returns the value associated with element x.

            @param[in]  x   The element whose value to return
            @pre
                -   x must be an element in the disjoint set forest
            @throw Exception
                -   If the precondition is violated
            @return As described
            */
            T &value_of(int x)
            {
                return get_element(x).m_value;
            }

            /**
            @brief  Returns the value associated with element x.

            @param[in]  x   The element whose value to return
            @pre
                -   x must be an element in the disjoint set forest
            @throw Exception
                -   If the precondition is violated
            @return As described
            */
            const T &value_of(int x) const
            {
                return get_element(x).m_value;
            }

            //#################### PRIVATE METHODS ####################
        private:
            Element &get_element(int x) const
            {
                typename std::map<int, Element>::iterator it = m_elements.find(x);
                if (it != m_elements.end())
                    return it->second;
                else
                {
                    printf("[FATAL ERROR, you tried to get_element( %d)  with nonexiting\n", x);
                    // std::terminate();
                }
                // return nullptr;
                // throw Exception(OSSWrapper() << "No such element: " << x);
            }

            void link(int x, int y)
            {
                Element &elementX = get_element(x);
                Element &elementY = get_element(y);
                int &rankX = elementX.m_rank;
                int &rankY = elementY.m_rank;
                if (rankX > rankY)
                {
                    elementY.m_parent = x;
                }
                else
                {
                    elementX.m_parent = y;
                    if (rankX == rankY)
                        ++rankY;
                }
                --m_setCount;
            }
        };

    }

    template <typename T>
    class DisjointSet
    {
        // note: since our data structure is std::map, if we add an item
        // 2 times it is overwritten. This might have some unwanted effect for
        // feature tracks.

    private:
        struct Node
        {
            int rank;
            T attribute;
            std::unique_ptr<Node> parent;

            // Default constructor
            Node() : rank(0), parent(nullptr) {}

            // Constructor with a value
            Node(const T &value) : rank(0), attribute(value), parent(nullptr) {}
        };

        std::map<T, std::unique_ptr<Node>> elementMap;

    public:
        // Constructor with member initializer list to initialize elementMap
        DisjointSet() : elementMap() {}

// Make a set with a single element.
#if 0
        void make_set(const T &value)
        {
            auto node = std::make_unique<Node>(value);
            elementMap[value] = std::move(node);
        }
#endif

        // add item only when it does not exisits
        void make_set(const T &value)
        {
            // Check if the element already exists in the map
            if (true || elementMap.find(value) == elementMap.end())
            {
                auto node = std::make_unique<Node>(value);
                elementMap[value] = std::move(node);
            }
            else
            {
                printf("adding an already existing item\n");
            }
        }

        // Find the representative element of the set to which x belongs.
        Node *find(const T &value)
        {
            Node *node = elementMap[value].get();
            if (node->parent && node->parent.get() != node)
            {
                // Path compression: Make every ancestor of x point directly to the representative.
                node->parent = std::unique_ptr<Node>(find(node->parent->attribute));
            }
            return node->parent.get() ? node->parent.get() : node;
        }

        const Node *find(const T &value) const
        {
            const Node *node = elementMap.at(value).get();
            while (node->parent && node->parent.get() != node)
            {
                node = node->parent.get();
            }
            return node;
        }

        // Union the sets to which x and y belong.
        void unionSets(const T &x, const T &y)
        {
            Node *rootX = find(x);
            Node *rootY = find(y);

            // Union by rank: Attach the shorter tree to the root of the taller tree.
            if (rootX != rootY)
            {
                if (rootX->rank < rootY->rank)
                {
                    rootX->parent.release(); // Release ownership of the old parent
                    rootX->parent = std::make_unique<Node>(rootY->attribute);
                }
                else if (rootX->rank > rootY->rank)
                {
                    rootY->parent.release(); // Release ownership of the old parent
                    rootY->parent = std::make_unique<Node>(rootX->attribute);
                }
                else
                {
                    rootY->parent.release(); // Release ownership of the old parent
                    rootY->parent = std::make_unique<Node>(rootX->attribute);
                    rootX->rank++;
                }
            }
        }

        // Get the attribute of the element x.
        const T &getAttribute(const T &value) const
        {
            return elementMap.at(value)->attribute;
        }

        // Count the total number of sets
        int CountSets() const
        {
            int count = 0;
            for (const auto &pair : elementMap)
            {
                if (!pair.second->parent || pair.second->parent.get() == pair.second.get())
                {
                    // The element is the representative of its set
                    count++;
                }
            }
            return count;
        }

        std::vector<T> GetAllRoots() const
        {
            std::vector<T> items;
            for (const auto &pair : elementMap)
            {
                if (!pair.second->parent || pair.second->parent.get() == pair.second.get())
                {
                    // The element is the representative of its set
                    items.push_back(pair.first);
                }
            }
            return items;
        }

        // Get the root (representative) of the set to which x belongs.
        T GetRoot(const T &value) const
        {
            Node *root = find(value);
            return (!root->parent || root->parent.get() == root) ? root->attribute : root->parent->attribute;
        }

        // Get all items of the set as a std::vector
        std::vector<T> GetSetItems(const T &value) const
        {
            const Node *root = find(value);

            // Traverse the set and collect items in a vector
            std::vector<T> items;
            for (const auto &pair : elementMap)
            {
                if (!pair.second->parent || find(pair.first) == root)
                {
                    items.push_back(pair.first);
                }
            }

            return items;
        }
    };

} // namespace datastruct