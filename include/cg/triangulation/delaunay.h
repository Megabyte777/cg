#pragma once
#include <cg/operations/contains/triangle_point.h>

#include <vector>
#include <algorithm>
#include <memory>

namespace cg
{
    inline bool delaunay_criterion(point_2 const&, point_2 const&, point_2 const&, point_2 const&);

    template <typename Scalar>
    class triangulation
    {
        struct node
        {
            point_2t<Scalar> p;
            bool infinite;

            node(point_2t<Scalar> p, bool infinite = false)
                : p(p)
                , infinite(infinite)
            {}

            point_2t<Scalar> geometry() const
            {
                assert(!infinite);
                return p;
            }
        };

        struct face;

        struct edge
        {
            std::shared_ptr<node> a, b;
            std::shared_ptr<edge> twin;
            std::shared_ptr<edge> next;
            std::shared_ptr<face> f;

            segment_2t<Scalar> geometry() const
            {
                return segment_2t<Scalar>(a->geometry(), b->geometry());
            }

            bool is_left(std::shared_ptr<node> np) const
            {
                assert(!a->infinite && !b->infinite);
                if (np->infinite)
                    return false;
                segment_2t<Scalar> seg = geometry();
                orientation_t orient = orientation(seg[0], seg[1], np->geometry());
                return (orient == CG_LEFT) || (orient == CG_COLLINEAR);
            }

            bool infinite() const
            {
                return a->infinite || b->infinite;
            }

            void flip()
            {
                auto a = this->a;
                auto b = this->b;
                auto c = next->b;
                auto d = twin->next->b;
                auto ab = twin->twin;
                auto bc = ab->next;
                auto ca = bc->next;
                auto ba = twin;
                auto ad = ba->next;
                auto db = ad->next;
                ab->a = d;
                ab->b = c;
                ab->next = ca;
                ca->next = ad;
                ad->next = ab;
                ad->f = ab->f;
                ba->a = c;
                ba->b = d;
                ba->next = db;
                db->next = bc;
                bc->next = ba;
                bc->f = ba->f;
                ab->f->e = ab;
                ba->f->e = ba;
            }

            bool bad() const
            {
                if (twin->next->b->infinite)
                    return false;
                if (f->infinite())
                {
                    if (!infinite())
                        return false;
                    auto ep = twin->twin;
                    while (ep->infinite())
                        ep = ep->next;
                    return ep->is_left(twin->next->b);
                }
                else
                {
                    triangle_2t<Scalar> tr = f->geometry();
                    return !delaunay_criterion(tr[0], tr[1], tr[2], twin->next->b->geometry());
                }
            }
        };

        struct face
        {
            std::shared_ptr<edge> e;

            triangle_2t<Scalar> geometry() const
            {
                return triangle_2t<Scalar>(e->a->geometry(),
                                           e->b->geometry(),
                                           e->next->b->geometry());
            }

            bool contains(std::shared_ptr<node> np) const
            {
                if (infinite())
                {
                    auto ep = e;
                    while (ep->infinite())
                        ep = ep->next;
                    return ep->is_left(np);
                }
                auto ep = e;
                for (int i = 0; i < 3; ++i)
                {
                    if (!ep->is_left(np))
                        return false;
                    ep = ep->next;
                }
                return true;
            }

            bool infinite() const
            {
                auto ep = e;
                for (int i = 0; i < 3; ++i)
                {
                    if (ep->a->infinite)
                        return true;
                    ep = ep->next;
                }
                return false;
            }

            void check_criterion()
            {
                auto ep = e;
                for (int i = 0; i < 3; ++i)
                {
                    if (ep->bad())
                    {
                        ep->flip();
                        if (auto ep1 = ep->next->twin)
                            ep1->f->check_criterion();
                        if (auto ep1 = ep->next->next->twin)
                            ep1->f->check_criterion();
                        if (auto ep1 = ep->twin->next->twin)
                            ep1->f->check_criterion();
                        if (ep->twin)
                            if (auto ep1 = ep->twin->next->next->twin)
                                ep1->f->check_criterion();
                        break;
                    }
                    ep = ep->next;
                }
            }
        };

        std::vector<std::shared_ptr<node> > nodes;
        std::vector<std::shared_ptr<edge> > edges;
        std::vector<std::shared_ptr<face> > faces;

        void init()
        {
            if (nodes.size() != 2 || !edges.empty() || !faces.empty())
                return;
            auto np = std::shared_ptr<node>(new node(point_2t<Scalar>(0, 0), true));
            nodes.push_back(np);
            for (int i = 0; i < 6; ++i)
                edges.push_back(std::shared_ptr<edge>(new edge()));
            for (int i = 0; i < 2; ++i)
                faces.push_back(std::shared_ptr<face>(new face()));
            for (int i = 0; i < 3; ++i)
            {
                edges[i]->a = nodes[i];
                edges[i]->b = nodes[(i + 1) % 3];
                edges[i]->next = edges[(i + 1) % 3];
                edges[i]->f = faces[0];

                edges[3 + i]->a = *(nodes.rbegin() + i);
                edges[3 + i]->b = *(nodes.rbegin() + (i + 1) % 3);
                edges[3 + i]->next = edges[3 + (i + 1) % 3];
                edges[3 + i]->f = faces[1];
            }
            edges[0]->twin = edges[4];
            edges[4]->twin = edges[0];
            edges[1]->twin = edges[3];
            edges[3]->twin = edges[1];
            edges[2]->twin = edges[5];
            edges[5]->twin = edges[2];
            faces[0]->e = edges[0];
            faces[1]->e = edges[4];
        }

        void insert_node_into_face(std::shared_ptr<node> np, std::shared_ptr<face> fp)
        {
            std::shared_ptr<edge> ep = fp->e;
            for (int i = 0; i < 3; ++i)
            {
                std::shared_ptr<face> nfp(new face());
                ep->f = nfp;
                std::shared_ptr<edge> nep1(new edge());
                nep1->a = ep->b;
                nep1->b = np;
                nep1->f = nfp;
                std::shared_ptr<edge> nep2(new edge());
                nep2->a = np;
                nep2->b = ep->a;
                nep2->f = nfp;
                std::shared_ptr<edge> nep = ep->next;
                ep->next = nep1;
                nep1->next = nep2;
                nep2->next = ep;

                edges.push_back(nep1);
                edges.push_back(nep2);
                nfp->e = ep;
                faces.push_back(nfp);
                ep = nep;
            }
            for (int i = 0; i < 3; ++i)
            {
                std::shared_ptr<face> fp1 = *(faces.rbegin() + (i + 1) % 3);
                std::shared_ptr<face> fp2 = *(faces.rbegin() + i);
                std::shared_ptr<edge> ep1 = fp1->e->next;
                std::shared_ptr<edge> ep2 = fp2->e->next->next;
                ep1->twin = ep2;
                ep2->twin = ep1;
            }
            for (int i = 0; i < 3; ++i)
                faces[faces.size() - i - 1]->check_criterion();
        }

    public:
        void clear()
        {
            for (std::shared_ptr<node> np : nodes)
                np.reset();
            nodes.clear();
            for (std::shared_ptr<edge> ep : edges)
                ep.reset();
            edges.clear();
            for (std::shared_ptr<face> fp : faces)
                fp.reset();
            faces.clear();
        }

        void add_point(point_2t<Scalar> const &p)
        {
            if (std::find_if(nodes.begin(), nodes.end(),
                    [p] (std::shared_ptr<node> np) { return !np->infinite && np->geometry() == p; }) != nodes.end())
                return;
            std::shared_ptr<node> np(new node(p));
            nodes.push_back(np);
            if (nodes.size() < 3)
            {
                if (nodes.size() == 2)
                    init();
                return;
            }
            auto f = std::find_if(faces.begin(), faces.end(),
                         [np] (std::shared_ptr<face> f)
                         {
                             return f->contains(np);
                         });
            assert(f != faces.end());
            std::shared_ptr<face> fp = *f;
            *f = faces.back();
            faces.pop_back();
            insert_node_into_face(np, fp);
        }

        std::vector<triangle_2t<Scalar> > get_triangles() const
        {
            std::vector<triangle_2t<Scalar> > res;
            for (std::shared_ptr<face> f : faces)
                if (!f->infinite())
                    res.push_back(f->geometry());
            return res;
        }

        ~triangulation()
        {
            clear();
        }
    };

    template <typename Scalar>
    Scalar determinant(Scalar a[3][3])
    {
        return   a[0][0] * a[1][1] * a[2][2]
               + a[0][1] * a[1][2] * a[2][0]
               + a[0][2] * a[1][0] * a[2][1]
               - a[0][2] * a[1][1] * a[2][0]
               - a[0][1] * a[1][0] * a[2][2]
               - a[0][0] * a[1][2] * a[2][1];
    }

    template <typename Scalar>
    Scalar delaunay_criterion_func(point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d)
    {
        Scalar m[3][3] = {{Scalar(a.x) - d.x, Scalar(a.y) - d.y, (Scalar(a.x) * a.x - Scalar(d.x) * d.x) + (Scalar(a.y) * a.y - Scalar(d.y) * d.y)},
                          {Scalar(b.x) - d.x, Scalar(b.y) - d.y, (Scalar(b.x) * b.x - Scalar(d.x) * d.x) + (Scalar(b.y) * b.y - Scalar(d.y) * d.y)},
                          {Scalar(c.x) - d.x, Scalar(c.y) - d.y, (Scalar(c.x) * c.x - Scalar(d.x) * d.x) + (Scalar(c.y) * c.y - Scalar(d.y) * d.y)}};
        return determinant(m);
    }

    struct delaunay_criterion_d
    {
        boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const
        {
            double res = delaunay_criterion_func<double>(a, b, c, d);
            double eps = res * 16 * std::numeric_limits<double>::epsilon();
            if (res > eps)
                return false;

            if (res < -eps)
                return true;

            return boost::none;
        }
    };

    struct delaunay_criterion_i
    {
        boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const
        {
            typedef boost::numeric::interval_lib::unprotect<boost::numeric::interval<double> >::type interval;

            boost::numeric::interval<double>::traits_type::rounding _;

            interval res = delaunay_criterion_func<interval>(a, b, c, d);

            if (res.lower() > 0)
                return false;

            if (res.upper() < 0)
                return true;

            if (res.upper() == res.lower())
                return true;

            return boost::none;
        }
    };

    struct delaunay_criterion_r
    {
        boost::optional<bool> operator() (point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d) const
        {
            mpq_class res = delaunay_criterion_func<mpq_class>(a, b, c, d);

            int cres = cmp(res, 0);

            return cres <= 0;
        }
    };

    inline bool delaunay_criterion(point_2 const & a, point_2 const & b, point_2 const & c, point_2 const & d)
    {
        if (boost::optional<bool> v = delaunay_criterion_d()(a, b, c, d))
            return *v;

        if (boost::optional<bool> v = delaunay_criterion_i()(a, b, c, d))
            return *v;

       return *delaunay_criterion_r()(a, b, c, d);
    }
}
