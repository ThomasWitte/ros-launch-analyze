#define _GNU_SOURCE 1

#include <dlfcn.h>
#include <ros/ros.h>
#include <iostream>

template<typename memberT>
union u_ptm_cast {
    memberT pmember;
    struct vstruct {void *pvoid; char padding[8];} vs;

    static_assert(sizeof(memberT) == sizeof(vstruct), "void* and member* must have the same size");
};

namespace ros {

Publisher NodeHandle::advertise(AdvertiseOptions& opts) {
    typedef Publisher (NodeHandle::*advertise_t)(AdvertiseOptions&);

    static advertise_t orig_advertise = nullptr;

    if (!orig_advertise) {
        u_ptm_cast<advertise_t> tmp;
        tmp.pmember = nullptr;
        tmp.vs.pvoid = dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9advertiseERNS_16AdvertiseOptionsE");
        orig_advertise = tmp.pmember;
    }

    std::cout << "<<advertise>> " << opts.topic << " " << opts.datatype << std::endl;

    return (this->*orig_advertise)(opts);
}

Subscriber NodeHandle::subscribe(SubscribeOptions& opts) {
    typedef Subscriber (NodeHandle::*subscribe_t)(SubscribeOptions&);

    static subscribe_t orig_subscribe = nullptr;

    if (!orig_subscribe) {
        u_ptm_cast<subscribe_t> tmp;
        tmp.pmember = nullptr;
        tmp.vs.pvoid = dlsym(RTLD_NEXT, "_ZN3ros10NodeHandle9subscribeERNS_16SubscribeOptionsE");
        orig_subscribe = tmp.pmember;
    }

    std::cout << "<<subscribe>> " << opts.topic << " " << opts.datatype << std::endl;

    return (this->*orig_subscribe)(opts);
}

}
