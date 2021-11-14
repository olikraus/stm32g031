
#include <stdint.h>

static const char *u16toap(char * dest, uint16_t v)
{
  uint8_t pos;
  uint8_t d;
  uint16_t c;
  c = 10000;
  for( pos = 0; pos < 5; pos++ )
  {
      d = '0';
      while( v >= c )
      {
	v -= c;
	d++;
      }
      dest[pos] = d;
      c /= 10;
  }  
  dest[5] = '\0';
  return dest;
}

/* convert unsigned 16 bit value to decimal number */
const char *u16toa(uint16_t v)
{
  static char buf[6];
  const char *s = u16toap(buf, v);
  while( *s == '0' )
    s++;
  if ( *s == '\0' )
    s--;
  return s;
}


static const char *u32toap(char * dest, uint32_t v)
{
  uint8_t pos;
  uint8_t d;
  uint32_t c;
  c = 1000000000UL;
  for( pos = 0; pos < 10; pos++ )
  {
      d = '0';
      while( v >= c )
      {
	v -= c;
	d++;
      }
      dest[pos] = d;
      c /= 10;
  }  
  dest[11] = '\0';
  return dest;
}

const char *u32toa(uint32_t v)
{
  static char buf[12];
  const char *s = u32toap(buf, v);
  while( *s == '0' )
    s++;
  if ( *s == '\0' )
    s--;
  return s;
}
