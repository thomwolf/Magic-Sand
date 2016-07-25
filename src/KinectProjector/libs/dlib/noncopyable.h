#ifndef DLIB_BOOST_NONCOPYABLE_HPP_INCLUDED
#define DLIB_BOOST_NONCOPYABLE_HPP_INCLUDED
namespace dlib {
    class noncopyable {
    protected:
        noncopyable() {}
        ~noncopyable() {}
    private:
        noncopyable(const noncopyable&);
        const noncopyable& operator=(const noncopyable&);
    };
}
#endif  // DLIB_BOOST_NONCOPYABLE_HPP_INCLUDED