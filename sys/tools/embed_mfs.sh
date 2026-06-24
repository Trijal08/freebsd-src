#!/bin/sh
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2008 The FreeBSD Project. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
#
# Embed an MFS image into the kernel body or the loader body (expects space
# reserved via MD_ROOT_SIZE (kernel) or MD_IMAGE_SIZE (loader))
#
# $1: kernel or loader filename
# $2: MFS image filename
#

if [ $# -ne 2 ]; then
	echo "usage: $(basename $0) target mfs_image"
	exit 0
fi
if [ ! -w "$1" ]; then
	echo $1 not writable
	exit 1
fi

# Portable MFS image size (BSD stat -f vs GNU stat -c differ; wc -c is both).
mfs_size=`wc -c < "$2" 2> /dev/null | tr -d '[:space:]'`
# If we can't determine MFS image size - bail.
if [ -z "${mfs_size}" ]; then
	echo "Can't determine MFS image size"
	exit 1
fi

err_no_mfs="Can't locate mfs section within "

# Find the reserved space for the image.  An ELF kernel carries it as the
# "oldmfs" section; a raw loader image instead carries string markers.  Use
# elfdump (or readelf), avoiding file(1)/od(1) which are not present in the
# build PATH when cross-building on a non-FreeBSD host.
sec_start=
sec_size=
if command -v elfdump > /dev/null 2>&1; then
	sec_info=`elfdump -c "$1" 2> /dev/null | grep -A 5 -E "sh_name: oldmfs$"`
	if [ -n "${sec_info}" ]; then
		sec_start=`echo "${sec_info}" | awk '/sh_offset/ {print $2}'`
		sec_size=`echo "${sec_info}" | awk '/sh_size/ {print $2}'`
	fi
fi
if [ -z "${sec_start}" ]; then
	rdelf=`command -v readelf 2> /dev/null || command -v llvm-readelf 2> /dev/null`
	if [ -n "${rdelf}" ]; then
		# Strip the "[Nr]" column, read Off and Size (hex) of oldmfs.
		sec_info=`"${rdelf}" -SW "$1" 2> /dev/null | \
		    sed -E 's/\[[ 0-9]*\]//' | \
		    awk '$1=="oldmfs" {print "0x"$4, "0x"$5; exit}'`
		if [ -n "${sec_info}" ]; then
			sec_start=`echo ${sec_info} | cut -d' ' -f1`
			sec_size=`echo ${sec_info} | cut -d' ' -f2`
		fi
	fi
fi
if [ -n "${sec_start}" ]; then
	# Normalise (hex or decimal) to a plain decimal byte count.
	sec_start=`printf '%d' "${sec_start}" 2> /dev/null`
	sec_size=`printf '%d' "${sec_size}" 2> /dev/null`
else
	# Raw image: locate the markers with strings.
	sec_start=`strings -at d "$1" | grep "MFS Filesystem goes here"` || \
	    { echo "${err_no_mfs} $1"; exit 1; }
	sec_start=`echo ${sec_start} | awk '{print $1}'`

	sec_end=`strings -at d "$1" | \
	    grep "MFS Filesystem had better STOP here"` || \
	    { echo "${err_no_mfs} $1"; exit 1; }
	sec_end=`echo ${sec_end} | awk '{print $1}'`

	sec_size=`expr ${sec_end} - ${sec_start}`
fi

# If the mfs section size is smaller than the mfs image - bail.
if [ ${sec_size} -lt ${mfs_size} ]; then
	echo "MFS image too large"
	exit 1
fi

# Dump the mfs image into the mfs section
dd if=$2 ibs=8192 of=$1 obs=${sec_start} seek=1 conv=notrunc 2> /dev/null && \
    echo "MFS image embedded into $1" && exit 0
