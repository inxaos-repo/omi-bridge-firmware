CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined
SRC = ../../components/omi_proto/omi_proto.c test_omi_proto.c

test: test_omi_proto
	./test_omi_proto

test_omi_proto: $(SRC) ../../components/omi_proto/include/omi_proto.h
	$(CC) $(CFLAGS) -I../../components/omi_proto/include $(SRC) -o $@

clean:
	rm -f test_omi_proto

.PHONY: test clean
