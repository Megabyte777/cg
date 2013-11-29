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

            std::ostream & operator << (std::ostream & out)
            {
                out << "(" << p.x << ", " << p.y << ")";
                return out;
            }
        };

        struct face;

        struct edge
        {
            std::weak_ptr<node> a_, b_;
            std::weak_ptr<edge> twin_;
            std::weak_ptr<edge> next_;
            std::weak_ptr<face> f_;

            std::shared_ptr<node> a() const { return a_.lock(); }
            std::shared_ptr<node> b() const { return b_.lock(); }
            std::shared_ptr<edge> twin() const { return twin_.lock(); }
            std::shared_ptr<edge> next() const { return next_.lock(); }
            std::shared_ptr<face> f() const { return f_.lock(); }

            segment_2t<Scalar> geometry() const
            {
                return segment_2t<Scalar>(a()->geometry(), b()->geometry());
            }

            bool is_left(std::shared_ptr<node> np) const
            {
                assert(!a()->infinite && !b()->infinite);
                if (np->infinite)
                    return true;
                segment_2t<Scalar> seg = geometry();
                orientation_t orient = orientation(seg[0], seg[1], np->geometry());
                return (orient == CG_LEFT) || (orient == CG_COLLINEAR);
            }

            bool infinite() const
            {
                return a()->infinite || b()->infinite;
            }

            void flip()
            {
                std::shared_ptr<node> c = next()->b();
                std::shared_ptr<node> d = twin()->next()->b();
                std::shared_ptr<edge> ab = twin()->twin();
                std::shared_ptr<edge> bc = ab->next();
                std::shared_ptr<edge> ca = bc->next();
                std::shared_ptr<edge> ba = twin();
                std::shared_ptr<edge> ad = ba->next();
                std::shared_ptr<edge> db = ad->next();
                ab->a_ = d;
                ab->b_ = c;
                ab->next_ = ca;
                ca->next_ = ad;
                ad->next_ = ab;
                ad->f_ = ab->f();
                ba->a_ = c;
                ba->b_ = d;
                ba->next_ = db;
                db->next_ = bc;
                bc->next_ = ba;
                bc->f_ = ba->f();
                ab->f()->e_ = ab;
                ba->f()->e_ = ba;
            }

            bool bad() const
            {
                if (!f()->infinite() && !is_left(next()->b()))
                    return true;
                if (twin()->next()->b()->infinite)
                    return false;
                if (f()->infinite())
                {
                    auto ep = twin()->twin();
                    while (ep->infinite())
                        ep = ep->next();
                    return ep->is_left(twin()->next()->b());
                }
                else
                {
                    triangle_2t<Scalar> tr = f()->geometry();
                    return !delaunay_criterion(tr[0], tr[1], tr[2], twin()->next()->b()->geometry());
                }
            }

            void check_criterion()
            {
                if (bad())
                {
                    flip();
                    next()->check_criterion();
                    next()->next()->check_criterion();
                    twin()->next()->check_criterion();
                    twin()->next()->next()->check_criterion();
                }
            }
        };

        struct face
        {
            std::weak_ptr<edge> e_;

            std::shared_ptr<edge> e() const { return e_.lock(); }

            std::ostream & operator << (std::ostream & out)
            {
                out << *e()->a() << " " << *e()->b() << " " << *e()->next()->b();
                return out;
            }

            triangle_2t<Scalar> geometry() const
            {
                return triangle_2t<Scalar>(e()->a()->geometry(),
                                           e()->b()->geometry(),
                                           e()->next()->b()->geometry());
            }

            bool contains(std::shared_ptr<node> np) const
            {
                if (infinite())
                {
                    auto ep = e();
                    while (ep->infinite())
                        ep = ep->next();
                    return ep->is_left(np);
                }
                auto ep = e();
                for (int i = 0; i < 3; ++i)
                {
                    if (!ep->is_left(np))
                        return false;
                    ep = ep->next();
                }
                return true;
            }

            bool infinite() const
            {
                auto ep = e();
                for (int i = 0; i < 3; ++i)
                {
                    if (ep->a()->infinite)
                        return true;
                    ep = ep->next();
                }
                return false;
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
                edges[i]->a_ = nodes[i];
                edges[i]->b_ = nodes[(i + 1) % 3];
                edges[i]->next_ = edges[(i + 1) % 3];
                edges[i]->f_ = faces[0];

                edges[3 + i]->a_ = *(nodes.rbegin() + i);
                edges[3 + i]->b_ = *(nodes.rbegin() + (i + 1) % 3);
                edges[3 + i]->next_ = edges[3 + (i + 1) % 3];
                edges[3 + i]->f_ = faces[1];
            }
            edges[0]->twin_ = edges[4];
            edges[4]->twin_ = edges[0];
            edges[1]->twin_ = edges[3];
            edges[3]->twin_ = edges[1];
            edges[2]->twin_ = edges[5];
            edges[5]->twin_ = edges[2];
            faces[0]->e_ = edges[0];
            faces[1]->e_ = edges[4];
        }

        void insert_node_into_face(std::shared_ptr<node> np, std::shared_ptr<face> fp)
        {
            std::shared_ptr<edge> ep = fp->e();
            for (int i = 0; i < 3; ++i)
            {
                std::shared_ptr<face> nfp(new face());
                ep->f_ = nfp;
                std::shared_ptr<edge> nep1(new edge());
                nep1->a_ = ep->b();
                nep1->b_ = np;
                nep1->f_ = nfp;
                std::shared_ptr<edge> nep2(new edge());
                nep2->a_ = np;
                nep2->b_ = ep->a();
                nep2->f_ = nfp;
                std::shared_ptr<edge> nep = ep->next();
                ep->next_ = nep1;
                nep1->next_ = nep2;
                nep2->next_ = ep;

                edges.push_back(nep1);
                edges.push_back(nep2);
                nfp->e_ = ep;
                faces.push_back(nfp);
                ep = nep;
            }
            for (int i = 0; i < 3; ++i)
            {
                std::shared_ptr<face> fp1 = *(faces.rbegin() + (i + 1) % 3);
                std::shared_ptr<face> fp2 = *(faces.rbegin() + i);
                std::shared_ptr<edge> ep1 = fp1->e()->next();
                std::shared_ptr<edge> ep2 = fp2->e()->next()->next();
                ep1->twin_ = ep2;
                ep2->twin_ = ep1;
            }
            for (int i = 0; i < 3; ++i)
                faces[faces.size() - i - 1]->e()->check_criterion();
        }

    public:
        void clear()
        {
            nodes.clear();
            edges.clear();
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

        void remove_point(point_2t<Scalar> const &p)
        {
            auto npi = std::find_if(nodes.begin(), nodes.end(),
                                    [p] (std::shared_ptr<node> np)
                                    {
                                        return !np->infinite && np->geometry() == p;
                                    });
            if (npi == nodes.end())
                return;
            std::shared_ptr<node> np = *npi;
            *npi = nodes.back();
            nodes.pop_back();
            if (nodes.size() < 3)
            {
                if (nodes.size() == 2)
                {
                    node *n = 0;
                    if (!nodes[0]->infinite)
                        n = new node(nodes[0]->p);
                    else
                        n = new node(nodes[1]->p);
                    clear();
                    nodes.push_back(std::shared_ptr<node>(n));
                }
                return;
            }
            std::vector<std::shared_ptr<edge> > edges_;
            for (std::shared_ptr<edge> ep : edges)
                if (ep->a() == np)
                    edges_.push_back(ep);
            assert(edges_.size() > 2);
            for (int i = 0; i < edges_.size() - 3; ++i)
                edges_[i]->flip();
            std::shared_ptr<edge> ep;
            if (edges_.size() > 3)
                ep = edges_[edges_.size() - 4];
            else
                ep = edges_[0]->next();
            auto ep1 = ep->next()->twin()->next();
            auto ep2 = ep->next()->next()->twin()->next()->next();
            faces.erase(std::remove_if(faces.begin(), faces.end(),
                                        [ep1, ep2] (std::shared_ptr<face> fp) { return fp == ep1->f() || fp == ep2->f(); }),
                        faces.end());
            ep->next_ = ep1;
            ep1->next_ = ep2;
            ep2->next_ = ep;
            ep1->f_ = ep->f();
            ep2->f_ = ep->f();
            ep->f()->e_ = ep;
            edges.erase(std::remove_if(edges.begin(), edges.end(),
                                        [np] (std::shared_ptr<edge> ep) { return ep->a() == np || ep->b() == np; }),
                        edges.end());
            for (int i = 3; i < edges_.size(); ++i)
            {
                auto ep = *(edges_.rbegin() + i);
                if (ep->bad())
                    ep->flip();
            }
            ep->check_criterion();
            ep->next()->check_criterion();
            ep->next()->next()->check_criterion();
            for (int i = 0; i < edges_.size() - 3; ++i)
                edges_[i]->check_criterion();
        }

        std::vector<triangle_2t<Scalar> > get_triangles() const
        {
            std::vector<triangle_2t<Scalar> > res;
            for (std::shared_ptr<face> f : faces)
                if (!f->infinite())
                    res.push_back(f->geometry());
            return res;
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
