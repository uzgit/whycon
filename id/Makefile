CFLAGS := -ggdb -std=c++11 -Wall -O2 -march=native
CFLAGS += `Magick++-config --cxxflags --cppflags`
LDFLAGS := CNecklace.cpp -lm
LDFLAGS += `Magick++-config --ldflags --libs`

all: whycon_id_gen

whycon_id_gen: whycon_id_gen.cpp CNecklace.cpp
	$(CXX) whycon_id_gen.cpp $(CFLAGS) $(LDFLAGS) -o $@ 

whycode_unique_generator: whycode_unique_generator.cpp CNecklace.cpp
	$(CXX) whycode_unique_generator.cpp $(CFLAGS) $(LDFLAGS) -o $@ 

clean:
	rm -f *.o whycon_id_gen whycode_unique_generator

.PHONY: clean

