CFLAGS := -Wall `pkg-config --cflags libosmocore libosmogsm libosmotrau`
LIBS := `pkg-config --libs libosmocore libosmogsm libosmotrau`

all: trau2rtp

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $^

trau2rtp: trau2rtp.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	@rm -f *.o trau2rtp
