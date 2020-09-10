#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <string>
#include <bitset>
#include <vector>
#include <set>

using namespace std;

int ID_BITS = -1;

string decimal_to_binary(unsigned n){
        const int size=sizeof(n)*8;
        std::string res;
        bool s=0;
        for (int a=0;a<size;a++)
        {
                bool bit = n >> (size-1);
                if (bit)
                {
                        s=1;
                }

                if (s)
                {
                        res.push_back(bit+'0');
                }

                n <<= 1;
        }
        if( !res.size() )
        {
                res.push_back('0');
        }
        return res;
}

bool rotationally_self_asymmetric(uint64_t number, int bits)
{
	bool result = true;
	uint64_t rotated = number;
	
	int rotated_by = 0;
	while( (rotated_by < bits - 1) && result )
	{
                uint64_t first_bit = rotated & ((uint64_t)0b1);
                rotated = (rotated >> 1) | (first_bit << (ID_BITS*2 - 1));

		result = number != rotated;
		rotated_by ++;
	}

	return result;
}

bool is_valid_manchester_encoding(uint64_t data)
{
	uint64_t manchester_0 = 1;
	uint64_t manchester_1 = 2;

	uint64_t nibble_select = 0b11;

	int i = 0;
	bool valid = true;
	while( valid && (i < ID_BITS) )
	{
		valid = (( ( data & nibble_select ) == manchester_0 ) | ( (data & nibble_select ) == manchester_1 ));
		data = data >> 2;
		i ++;
	}

	return valid;
}

uint64_t rotate_to_minimal(const uint64_t number)
{
	uint64_t minimal = number;
	uint64_t rotated = number;

	for(int i = 0; i < ID_BITS * 2; i ++)
	{
		uint64_t first_bit = rotated & ((uint64_t)0b1);
		rotated = (rotated >> 1) | (first_bit << (ID_BITS*2 - 1));

		if( rotated < minimal && is_valid_manchester_encoding(rotated) )
		{
			minimal = rotated;
		}
	}

	return minimal;
}

uint64_t manchester_encode(uint64_t data)
{
	uint64_t manchester_0 = 1;
	uint64_t manchester_1 = 2;

	uint64_t bit_select = 1 << ID_BITS - 1;
	uint64_t encoding   = 0;
	for(int i = 0; i < ID_BITS; i ++)
	{
		encoding <<= 2;
		if( data & bit_select )
		{
			encoding |= manchester_1;
		}
		else
		{
			encoding |= manchester_0;
		}
		bit_select >>= 1;
	}

	return encoding;
}

uint64_t manchester_decode(uint64_t data)
{
	uint64_t manchester_0 = 1;
	uint64_t manchester_1 = 2;

	uint64_t zero_bit = 0b0;
	uint64_t one_bit  = 0b1;

	uint64_t decoding     = 0;
	uint64_t nibble_select = 0b11;
	for(int i = 0; i < ID_BITS; i ++)
	{
		if( ( data & nibble_select ) == manchester_0 )
		{
			decoding |= (zero_bit << i);
		}
		else if( ( data & nibble_select ) == manchester_1 )
		{
			decoding |= (one_bit << i);
		}
		else
		{
			// invalid
			throw runtime_error("Invalid Manchester encoding!");
		}
		data >>= 2;
	}
	
	return decoding;
}

string pad_zeros(string data, int bits)
{
	return string( bits - data.size(), '0').append( data );
}

string printable(unsigned n, int i = 1)
{
	return pad_zeros(decimal_to_binary(n), i*ID_BITS);
}

int main(int argc, char ** argv)
{
	if( argc != 2 )
	{
		cout << "Please enter the number of ID bits!" << endl;
		return -1;
	}

	ID_BITS = atoi(argv[1]);
	if( ID_BITS > 32 )
	{
		cout << "Please enter the number of ID bits < 16!" << endl;
		return -1;
	}

	// heading
//	cout << "ID\tBinary\tManchester\tMin Manchester\tMapped ID" << endl;
/*
	cout << setw(4)  << "ID"
	     << setw(8)  << "Binary"
	     << setw(25) << "Manchester"
	     << setw(25) << "Min Manchester"
	     << setw(15) << "Mapped ID"
	     << endl;
*/
	set<int> unique_ids;

	int max_id = pow(2, ID_BITS);
	for(int id = 0; id < max_id; id ++)
	{
		uint64_t manchester = manchester_encode(id);
		uint64_t minimal_manchester = rotate_to_minimal(manchester);
		uint64_t mapped_id  = manchester_decode(minimal_manchester);

		if( rotationally_self_asymmetric(minimal_manchester, 2*ID_BITS) )
		{
			unique_ids.insert(mapped_id);
		}
/*
		cout << setw(4)  << id
		     << setw(8)  << printable(id)
		     << setw(25) << printable(manchester, 2)
		     << setw(25) << printable(minimal_manchester, 2)
//		     << setw(15) << printable(mapped_id) << "="
		     << setw(15) << mapped_id
		     << endl;
*/
	}

	cout << unique_ids.size() << " unique IDs for " << ID_BITS << " bits:" << endl;;

	set<int>::iterator iterator = unique_ids.begin();
	cout << "{" << *iterator;

	if(unique_ids.size() > 1)
	{
		iterator ++;
		while( iterator != unique_ids.end() )
		{
			cout << ", " << *iterator;

			iterator ++;
		}
	}
	cout << "}" << endl;

	return 0;
}
