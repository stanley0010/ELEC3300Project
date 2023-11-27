/*
 * image.h
 *
 *  Created on: 31 ����. 2019 �.
 *      Author: tabur
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include <stdint.h>

typedef struct _tImage
{
  uint16_t 					xSize;
  uint16_t 					ySize;
  uint16_t 					bytesPerLine;
  uint8_t					bitsPerPixel;
  const unsigned char* 		pData;
} sImage;

#define GUI_BITMAP			sImage
#define GUI_CONST_STORAGE	const

extern GUI_CONST_STORAGE GUI_BITMAP peashooter_000_logo;
extern GUI_CONST_STORAGE GUI_BITMAP peashooter_001_logo;
extern GUI_CONST_STORAGE GUI_BITMAP peashooter_002_logo;
extern GUI_CONST_STORAGE GUI_BITMAP peashooter_003_logo;
extern GUI_CONST_STORAGE GUI_BITMAP peashooter_004_logo;
extern GUI_CONST_STORAGE GUI_BITMAP peashooter_005_logo;

#endif /* IMAGE_H_ */
