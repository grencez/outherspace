
CFLAGS = -g3 -ansi -pedantic -Wall -Wextra

main: main.o
	$(CC) $(CFLAGS) $^ -o $@

