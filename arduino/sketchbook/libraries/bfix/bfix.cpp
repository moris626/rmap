
//
//          Copyright Richard Allen Hogaboom 1997 - 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// Boost Software License - Version 1.0 - August 17th, 2003
//
// Permission is hereby granted, free of charge, to any person or organization
// obtaining a copy of the software and accompanying documentation covered by
// this license (the "Software") to use, reproduce, display, distribute,
// execute, and transmit the Software, and to prepare derivative works of the
// Software, and to permit third-parties to whom the Software is furnished to
// do so, all subject to the following:
//
// The copyright notices in the Software and this entire statement, including
// the above license grant, this restriction and the following disclaimer,
// must be included in all copies of the Software, in whole or in part, and
// all derivative works of the Software, unless such copies or derivative
// works are solely in the form of machine-executable object code generated by
// a source language processor.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//

/*
 *==================================================================================================
 *
 * File Name:
 *     bfix.cpp
 *
 *==================================================================================================
 *
 * Functions:
 *
 *         int
 *     bfi
 *         (
 *             unsigned char *cptr,
 *             unsigned long bit_offset,
 *             unsigned long bit_len,
 *             long value,
 *             unsigned int endian
 *         )
 *         e.g. int return = bfi(cptr, bit_offset, bit_len, value, endian);
 *
 *
 *         long
 *     bfx
 *         (
 *             const unsigned char *cptr,
 *             unsigned long bit_offset,
 *             unsigned long bit_len,
 *             unsigned int endian
 *         )
 *         e.g. long value = bfx(cptr, bit_offset, bit_len, endian);
 *
 *==================================================================================================
 *
 * Description:
 *
 *     bfi()/bfx() are used to insert and extract bit fields from an arbitrary length array
 *     of unsigned chars pointed to by an unsigned char* pointer.
 *
 *==================================================================================================
 *
 * Error Handling
 *
 *     1. Exceptions - None
 *     2. Debugging  - const int DEBUG = true for debugging output
 *     3. Returns
 *
 *            bfi():
 *                bit_offset < 1     - error, return -1
 *                bit_len < 1        - error, return -2
 *                bit_len > too long - error, return -3
 *                endian not 0-2     - error, return -4
 *                return 0           - success
 *
 *            bfx():
 *                bit_offset < 1     - error, return -1
 *                bit_len < 1        - error, return -2
 *                bit_len > too long - error, return -3
 *                endian not 0-2     - error, return -4
 *                return value       - success
 *
 *==================================================================================================
 *
 * Notes:
 *     1. in the following notes any annotation of the form n/m means n for 32 bit systems and
 *        m for 64 bit systems.  operation on 32 or 64 bit systems should be transparent.
 *
 *     2. bit_len should be <=32/64 to 25/57. it depends on the value of bit_offset.  the method
 *        always uses a memmove() of 4/8 bytes to a long temporary storage in which logical
 *        operations can be performed.  this means that the bit_len can be at most 4/8 bytes,
 *        but in a case in which the start bit is not at the beginning of a byte, then the
 *        bit_len can only extend until the end of the 4/8'th byte.  if the start bit is the
 *        last bit of a byte this will limit bit_len to 25/57 bits - the last bit of the first
 *        byte plus the next 3/7 bytes.
 *
 *     3. 4(32 bit machines)/8(64 bit machines) bytes are always read from the unsigned char
 *        array, modified and then written back.  this means that if you set the last bit of
 *        the array, then the next 3/7 bytes will be read and written back, thus seemingly
 *        overrunning the array.  if the 4/8 bytes does not overrun the array then no bits
 *        beyond the end of the array will be changed.  if the 4/8 bytes does overrun the
 *        array some provision must be made to deal with this possibility.  the array could
 *        be padded by 3/7 extra bytes.
 *
 *     4. bit_offset+bit_len should not overrun the array.
 *
 *     5. value should not be too long to fit into the bit field.  if it is, the high order bits
 *        in front of the low order bit_len bits will be truncated.
 *
 *     6. all bit_len bits will be set and no bit outside the bit field will be changed.
 *
 *     7. value may be negative and prefix 2's complement sign bits are truncated to fit into
 *        bit_len bits.
 *
 *     8. use the lscpu cmd to determine 32/64 bit and endianness:
 *        $ lscpu
 *        Architecture:          x86_64
 *        CPU op-mode(s):        32-bit, 64-bit
 *        Byte Order:            Little Endian
 *
 *==================================================================================================
 *
 * Author: Richard Hogaboom
 *         richard.hogaboom@gmail.com
 *
 *==================================================================================================
 */

#include "bfix.h"


/*
 *==================================================================================================
 *     bfi()
 *
 * Purpose: 
 *     extract bit field from an array of chars pointed to by an unsigned char* pointer
 *
 * Usage:
 *     unsigned char *cptr
 *     unsigned long bit_offset
 *     unsigned long bit_len
 *     long value
 *     unsigned int endian
 *
 *     int return = bfi(cptr, bit_offset, bit_len, value, endian);
 *
 * Returns:
 *     bit_offset < 1     - error, return -1
 *     bit_len < 1        - error, return -2
 *     bit_len > too long - error, return -3
 *     endian not 0-2     - error, return -4
 *     return 0           - success
 *
 * Parameters:
 *     unsigned char *cptr      - pointer to unsigned char array
 *     unsigned long bit_offset - bit offset(starting from 1) from the start of the char array
 *     unsigned long bit_len    - bit length of field to insert
 *     long value               - value to insert
 *     unsigned int endian      - endian enum, 0(run time checking), 1(big endian), 2(little endian)
 *
 * Comments:
 *     4(32 bit machines)/8(64 bit machines) bytes are always read from the unsigned char
 *     array, modified and then written back.  this means that if you set the last bit of
 *     the array, then the next 3/7 bytes will be read and written back, thus seemingly
 *     overrunning the array.  if the 4/8 bytes does not overrun the array then no bits
 *     beyond the end of the array will be changed.  if the 4/8 bytes does overrun the
 *     array some provision must be made to deal with this possibility.  the array could
 *     be padded by 3/7 extra bytes.
 */

    int
bfi
    (
        unsigned char *cptr,
        unsigned long bit_offset,
        unsigned long bit_len,
        long value,
        unsigned int endian
    )
{
    /* machine dependencies */
    const unsigned int BITS_PER_BYTE = 8;
    unsigned int BYTES_PER_LONG = sizeof(unsigned long);
    unsigned int BITS_PER_LONG = BYTES_PER_LONG * BITS_PER_BYTE;

    unsigned long l, m;
    unsigned int i, j, size;
    unsigned char* c = (unsigned char*)&l;
    unsigned char tmp;

    unsigned long mask;
    unsigned long byte_offset;
    unsigned long pre_shift;
    unsigned long post_shift;


    for ( i=0 ; i < BYTES_PER_LONG ; i++ )
    {
        ((unsigned char *)&mask)[i] = 0xff;
    }

    if ( bit_offset < 1 )
    {
      LOGD(  "bfi: bit_offset = %ld is < 1.\n", bit_offset);
      return -1;
    }

    if ( bit_len < 1 )
    {
      LOGD(  "bfi: bit_len = %ld is < 1.\n", bit_len);
      return -2;
    }

    /*
     * calculate byte offset(first byte=0) of start of
     * BYTES_PER_LONG bytes containing bit field
     */
    byte_offset = (bit_offset-1)/BITS_PER_BYTE;

    /*
     * calculate how many bits to shift bit field left
     * to clear bits above bit field
     */
    pre_shift = bit_offset - byte_offset*BITS_PER_BYTE - 1;

    if ( bit_len > (BITS_PER_LONG-pre_shift) )
    {
      LOGD(  "bfi: bit_len = %ld is too long.\n", bit_len);
      return -3;
    }

    /*
     * calculate how many bits to shift bit field left
     * to clear bits below bit field
     */
    post_shift = BITS_PER_LONG - bit_len - pre_shift;

    /*
     * move bit field into position over bits to set in l
     * corresponding to correct bits in cptr
     */
    value <<= post_shift;

    /* zero out mask bits after bit field */
    mask <<= post_shift;

    /* zero out mask bits before bit field */
    mask <<= pre_shift;
    mask >>= pre_shift;

    /* zero out value bits before and after bit field */
    value &= mask;

    /* move BYTES_PER_LONG bytes to tmp storage */
    memmove((unsigned char *)&l, &cptr[byte_offset], BYTES_PER_LONG);

    switch ( endian )
    {
        case 1:
            /* big endian */
            break;
        case 2:
            /* little endian */
            size = sizeof(l);
            for ( i = 0 ; i < size/2; i++ )
            {
                j = size - i - 1;
                tmp = c[i];
                c[i] = c[j];
                c[j] = tmp;
            }
            break;
        case 0:
            /* run time checking */
            m = l;
            l = 1;
            if (c[0])
            {
                /* little endian */
                l = m;
                size = sizeof(l);
                for ( i = 0 ; i < size/2; i++ )
                {
                    j = size - i - 1;
                    tmp = c[i];
                    c[i] = c[j];
                    c[j] = tmp;
                }
            }
            else
            {
                /* big endian */
                l = m;
            }
            break;

        default:
	  LOGD(  "bfi: endian = %d not 0-2.\n", endian);
	  return -4;
	  break;
    }

    /* zero out bit field bits and then or value bits into them */
    l = (l & (~mask)) | value;

    switch ( endian )
    {
        case 1:
            /* big endian */
            break;
        case 2:
            /* little endian */
            size = sizeof(l);
            for ( i = 0 ; i < size/2; i++ )
            {
                j = size - i - 1;
                tmp = c[i];
                c[i] = c[j];
                c[j] = tmp;
            }
            break;
        case 0:
            /* run time checking */
            m = l;
            l = 1;
            if (c[0])
            {
                /* little endian */
                l = m;
                size = sizeof(l);
                for ( i = 0 ; i < size/2; i++ )
                {
                    j = size - i - 1;
                    tmp = c[i];
                    c[i] = c[j];
                    c[j] = tmp;
                }
            }
            else
            {
                /* big endian */
                l = m;
            }
            break;

        default:
            break;
    }

    /* move tmp storage back to cptr array */
    memmove(&cptr[byte_offset], (unsigned char *)&l, BYTES_PER_LONG);

    return 0;
}


/*
 *==================================================================================================
 *     bfx()
 *
 * Purpose: 
 *     extract bit field from an array of chars pointed to by an unsigned char* pointer
 *
 * Usage:
 *     const unsigned char *cptr
 *     unsigned long bit_offset
 *     unsigned long bit_len
 *     unsigned int endian
 *
 *     long value = bfx(cptr, bit_offset, bit_len, endian);
 *
 * Returns:
 *     bit_offset < 1     - error, return -1
 *     bit_len < 1        - error, return -2
 *     bit_len > too long - error, return -3
 *     endian not 0-2     - error, return -4
 *     return value       - success
 *
 * Parameters:
 *     const unsigned char *cptr - const pointer to unsigned char array
 *     unsigned long bit_offset  - bit offset(starting from 1) from the start of the char array
 *     unsigned long bit_len     - bit length of field to extract
 *     unsigned int endian       - endian enum, 0(run time checking), 1(big endian), 2(little endian)
 *
 * Comments:
 */

    long
bfx
    (
        const unsigned char *cptr,
        unsigned long bit_offset,
        unsigned long bit_len,
        unsigned int endian
    )
{
    /* machine dependencies */
    const unsigned int BITS_PER_BYTE = 8;
    unsigned int BYTES_PER_LONG = sizeof(unsigned long);
    unsigned int BITS_PER_LONG = BYTES_PER_LONG * BITS_PER_BYTE;

    unsigned long l;
    unsigned int i, j, size;
    unsigned char* c = (unsigned char*)&l;
    unsigned char tmp;

    unsigned long byte_offset;
    unsigned long left_shift;
    unsigned long right_shift;


    if ( bit_offset < 1 )
    {
      LOGD(  "bfi: bit_offset = %ld is < 1.\n", bit_offset);
      return -1;
    }

    if ( bit_len < 1 )
    {
      LOGD(  "bfi: bit_len = %ld is < 1.\n", bit_len);
      return -2;
    }

    /*
     * calculate byte offset(first byte=0) of start of
     * BYTES_PER_LONG bytes containing bit field
     */
    byte_offset = (bit_offset-1)/BITS_PER_BYTE;

    /*
     * calculate how many bits to shift bit field left
     * to clear bits above bit field
     */
    left_shift = bit_offset - byte_offset*BITS_PER_BYTE - 1;

    if ( bit_len > (BITS_PER_LONG-left_shift) )
    {
      LOGD(  "bfi: bit_len = %ld is too long.\n", bit_len);
      return -3;
    }

    /*
     * calculate how many bits to shift bit field right
     * to right justify bit field
     */
    right_shift = BITS_PER_LONG - bit_len;

    switch ( endian )
    {
        case 1:
            /* big endian */
            /* move BYTES_PER_LONG bytes to tmp storage */
            memmove((unsigned char *)&l, &cptr[byte_offset], BYTES_PER_LONG);
            break;

        case 2:
            /* little endian */
            memmove((unsigned char *)&l, &cptr[byte_offset], BYTES_PER_LONG);
            size = sizeof(l);
            for ( i = 0 ; i < size/2; i++ )
            {
                j = size - i - 1;
                tmp = c[i];
                c[i] = c[j];
                c[j] = tmp;
            }
            break;

        case 0:
            /* run time checking */
            l = 1;
            if (c[0])
            {
                /* little endian */
                memmove((unsigned char *)&l, &cptr[byte_offset], BYTES_PER_LONG);
                size = sizeof(l);
                for ( i = 0 ; i < size/2; i++ )
                {
                    j = size - i - 1;
                    tmp = c[i];
                    c[i] = c[j];
                    c[j] = tmp;
                }
            }
            else
            {
                /* big endian */
                memmove((unsigned char *)&l, &cptr[byte_offset], BYTES_PER_LONG);
            }
            break;

        default:
	  LOGD(  "bfx: endian = %d not 0-2.\n", endian);
	  return -4;
	  break;
    }

    /*
     * clear bits above bit field, right justify bit
     * field, and return this value
     */
    return ( (l << left_shift) >> right_shift );
}

