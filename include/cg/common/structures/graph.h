#pragma once
#include <vector>

namespace cg
{
    class graph
    {
        size_t n;
        std::vector<std::vector<int> > edges;
    public:
        graph()
        {}

        graph(size_t n) :
            n(n), edges(n)
        {}

        size_t nodes_count() const
        {
            return n;
        }

        void add_edge(int x, int y)
        {
            edges[x].push_back(y);
        }

        void add_bidirected_edge(int x, int y)
        {
            add_edge(x, y);
            add_edge(y, x);
        }

        std::vector<int> const &get_edges(int x) const
        {
            return edges[x];
        }
    };
}
