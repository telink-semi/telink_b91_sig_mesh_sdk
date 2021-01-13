#ifndef AID__COMMON__C__
#define AID__COMMON__C__

#include <stdint.h>

void *aid_calloc (int n, int size)
{
	return malloc(n*size);
}

void *aid_malloc (int size)
{
	return malloc(size);
}

void *aid_realloc (void *ptr, int size)
{
	return realloc(ptr, size);
}

void aid_free (void *ptr)
{
	return free(ptr);
}

void aid_memset(char* buf, char data, unsigned int len)
{
	int i =0;
	for(i=0;i<len;i++)
	{
		buf[i] = data;
	}
	return;
}
#endif

