/********************************************************************************************************
 * @file	string.c
 *
 * @brief	for TLSR chips
 *
 * @author	BLE GROUP
 * @date	2020.06
 *
 * @par     Copyright (c) 2020, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *          
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *          
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *          
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions 
 *              in binary form must reproduce the above copyright notice, this list of 
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *          
 *              3. Neither the name of TELINK, nor the names of its contributors may be 
 *              used to endorse or promote products derived from this software without 
 *              specific prior written permission.
 *          
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or 
 *              relating to such deletion(s), modification(s) or alteration(s).
 *         
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *         
 *******************************************************************************************************/
#include "types.h"
#include "string.h"
#include "../common/assert.h"
#include "../tl_common.h"
#include "../drivers/9518/compiler.h"

_attribute_ram_code_ int tmemcmp(const void * m1, const void *m2, u32 len) {
	u8 *st1 = (u8 *) m1;
	u8 *st2 = (u8 *) m2;

	while(len--){
		if(*st1 != *st2){
			return (*st1 - *st2);
		}
		st1++;
		st2++;
	}
	return 0;
}



_attribute_ram_code_ void bbcopy(register char * src, register char * dest, int len) {
	if (dest < src)
		while (len--)
			*dest++ = *src++;
	else {
		char *lasts = src + (len - 1);
		char *lastd = dest + (len - 1);
		while (len--)
			*(char *) lastd-- = *(char *) lasts--;
	}
}

_attribute_ram_code_ void bcopy(register char * src, register char * dest, int len) {
	bbcopy(src, dest, len);
}

_attribute_ram_code_ void * tmemset(void * dest, int val, unsigned int len) {
	register unsigned char *ptr = (unsigned char*) dest;
	while (len-- > 0)
		*ptr++ = (unsigned char)val;
	return dest;
}

_attribute_ram_code_ void * tmemcpy(void * out, const void * in, unsigned int length) {
	bcopy((char *) in, (char *) out, (int) length);
	return out;
}

// for performance, assume lenght % 4 == 0,  and no memory overlapped
_attribute_ram_code_ void tmemcpy4(void * d, const void * s, unsigned int length){
	int* dst = (int*)d;
	int* src = (int*)s;
	assert((((int)dst) >> 2) << 2 == ((int)dst));			// address must alighn to 4
	assert((((int)src) >> 2) << 2 == ((int)src));			// address must alighn to 4
	assert((length >> 2) << 2 == length);					// lenght % 4 == 0
	assert(( ((char*)dst) + length <= (const char*)src) || (((const char*)src) + length <= (char*)dst));	//  no overlapped
	unsigned int len = length >> 2;
	while(len --){
		*dst++ = *src++;
	}
}

char *	strstr(const char *s1, const char *s2)
{
 int n;
 if (*s2){
  while(*s1){
    for (n=0;*(s1+n)==*(s2+n);n++){
	    if (!*(s2+n+1)){            //查找的下一个字符是否为'\0'
	     return (char*)s1;
	    }
    }
   	s1++;
  	}
  	return NULL;
 }else{
  return (char*)s1;
 }
}


int is_buf_zero(void *data, unsigned int len){
	u8 *p = (u8*)data;
	for(unsigned int i = 0; i < len; ++i){
		if(*p){
			return 0;
		}
		++p;
	}
	return 1;
}

int is_buf_all_one(void *data, unsigned int len){
	u8 *p = (u8*)data;
	for(unsigned int i = 0; i < len; ++i){
		if(*p != 0xff){
			return 0;
		}
		++p;
	}
	return 1;
}

int ismemzero4(void *data, unsigned int len){
	int *p = (int*)data;
	len = len >> 2;
	for(unsigned int i = 0; i < len; ++i){
		if(*p){
			return 0;
		}
		++p;
	}
	return 1;
}

int ismemf4(void *data, unsigned int len){
	int *p = (int*)data;
	len = len >> 2;
	for(unsigned int i = 0; i < len; ++i){
		if(*p != 0xffffffff){
			return 0;
		}
		++p;
	}
	return 1;
}

void * memset4(void * dest, int val, unsigned int len) {
	int *p = (int*)dest;
	len = len >> 2;
	for(unsigned int i = 0; i < len; ++i){
		*p++ = val;
	}
	return dest;
}

void zeromem4(void *data, unsigned int len){
	memset4(data, 0, len);
}


