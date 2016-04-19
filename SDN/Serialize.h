//
//  Serialize.h
//  SDN
//
//  Created by Gautam on 4/19/16.
//  Copyright © 2016 Gautam. All rights reserved.
//

#ifndef Serialize_h
#define Serialize_h

void unpack(unsigned char *buf, char *format, ...);
unsigned int pack(unsigned char *buf, char *format, ...);


#endif /* Serialize_h */
