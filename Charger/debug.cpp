#include "defs.h"
#include "stdarg.h"
#include "usi-serial.h"

void initDebug()
{
#ifdef MULTI_CHARGERR
    Serial.begin(19200);
#endif
#if defined(SINGLE_CHARGER) && defined(SERIAL_DEBUG)
	serialSetup();	
#endif
}

inline void debug_ch(char c)
{
#ifdef MULTI_CHARGERR
	Serial.print(c);
#endif
#if defined(SINGLE_CHARGER) && defined(SERIAL_DEBUG)
	serialWrite(c);	
#endif
}

void debug_str(const char* s)
{
	while (*s) debug_ch(*s++);
}

void debug_str_p(const char* s)
{
	char c = pgm_read_byte(s++);
	while (c) {
		debug_ch(c);
		c = pgm_read_byte(s++);
	}
}

static bool in_pct = false;
static va_list ap;

static void debugf_ch(char ch)
{
	if (!in_pct) {
    if (ch == '%') {
      in_pct = true;
    } else {
      if (ch == '\n') {
        debug_ch('\r');
      }
      debug_ch(ch);
    }
  } else {
		if (ch < '0' || ch > '9') {
			switch (ch) {
			case '.':
			case 'z':
			case 'l':
			case '-':
			case '#':
				// Ignore modifiers for now:
				break;
			case '%':
				debug_ch('%');
				in_pct = false;
				break;
			case 'd':
			case 'i':
				{
					char buf[10];
					char* p = buf+sizeof(buf);
					*--p = '\0';
					int value = va_arg(ap, int);
					if (value < 0) {
						debug_ch('-');
						value = -value;
					}
					while (value) {
						*--p = (value%10)+'0';
						value /= 10;
					}
					if (!*p) *--p = '0';
					debug_str(p);
				}
        in_pct = false;
				break;
      case 'u':
        {
          char buf[10];
          char* p = buf+sizeof(buf);
          *--p = '\0';
          unsigned int value = va_arg(ap, unsigned int);
          while (value) {
            *--p = (value%10)+'0';
            value /= 10;
          }
          if (!*p) *--p = '0';
          debug_str(p);
        }
        in_pct = false;
        break;
			case 'x':
				{
					char buf[10];
					char* p = buf+sizeof(buf);
					*--p = '\0';
					unsigned int value = va_arg(ap, unsigned int);
					while (value) {
						byte v = value & 0x0f;
						*--p = v<10 ? v+'0' : v+'a';
						value >>= 4;
					}
					if (!*p) *--p = '0';
					debug_str(p);
				}
        in_pct = false;
				break;
			case 'c':
				debug_ch(va_arg(ap, char));
        in_pct = false;
				break;
			case 's':
				debug_str(va_arg(ap, const char*));
        in_pct = false;
				break;
			case 'S':
				debug_str_p(va_arg(ap, const char*));
        in_pct = false;
				break;
			default:
				debug_ch('%');
				debug_ch(ch);
        in_pct = false;
        break;
			}
		}
	}
}

// Simple printf substitute taking progmem format:
void debugf_p(const char* progmem_fmt, ...)
{
	va_start(ap, progmem_fmt);
	byte ch = pgm_read_byte(progmem_fmt++);
	while (ch != '\0') {
		debugf_ch(ch);
		ch = pgm_read_byte(progmem_fmt++);
	}
	if (in_pct) {
		debug_ch('%');
		in_pct = false;
	}
}

void debugf(const char* fmt, ...)
{
	va_start(ap, fmt);
	byte ch = *fmt++;
	while (ch != '\0') {
		debugf_ch(ch);
		byte ch = *fmt++;
	}
	if (in_pct) {
		debug_ch('%');
		in_pct = false;
	}
}

