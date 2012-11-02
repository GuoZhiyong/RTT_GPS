#ifndef _UDISK_H_
#define _UDISK_H_


struct _udiskinst
{
    upipe_t pipe_in;
    upipe_t pipe_out;

    struct rt_device device;
};    
typedef struct _udiskinst* udiskinst_t;



struct _udiskmsgcontext
{
	uifinst_t ifinst;
	void *payload;
};

typedef struct _udiskmsgcontext* udisk_msgcontext_t;



#endif

