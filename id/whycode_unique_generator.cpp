#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <bitset>
#include <Magick++.h>
#include "CNecklace.h"
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

// coordinates_list contains the coordinate of polygons used to create the ID bits
// drawable_list contains the drawable items for Magick

using namespace Magick;
using namespace std;

std::vector<int> unique_ids;
std::vector<string> base_representations;

// Define default parameters
bool verbose = false;
bool legacy = false;
bool single = false;
int hamming = 1;
int x_center = 900;
int y_center = 900;
int index = 0;
int n;
float w;
bool draw_outer_border = true;
int id_bits = -1;

string background_color		= "white";
string inner_background_color	= background_color;
string border_color		= "black";
string inner_segment_color	= "white";
string outer_segment_color	= "black";

// the radius of the polygons used to create the ID (switch to a different color to get an idea of what these are)
int polygon_outer_radius = 650;
string polygon_color = outer_segment_color;

// whycode_skeleton is the blank WhyCon marker
Image whycode_skeleton;
//Image a4_grid_canvas;

void display_help()
{
    printf("\nUsage: whycode_unique_generator <# id bits>\n");
}

// Draw the original WhyCon marker
void draw_whycon_marker(Image &image)
{
	list<Drawable> drawable_list;
	image.size("1800x1800");
	image.backgroundColor(Color(background_color));
	image.erase();
	image.resolutionUnits(PixelsPerCentimeterResolution);
	image.density("300x300");

	drawable_list.push_back(DrawableStrokeColor(border_color));
	drawable_list.push_back(DrawableFillColor(background_color));
	if( draw_outer_border )
	{
		drawable_list.push_back(DrawableEllipse(x_center, y_center, 899, 899, 0, 360));
	}
	image.draw(drawable_list);
	drawable_list.clear();


	drawable_list.push_back(DrawableStrokeWidth(0));
	drawable_list.push_back(DrawableFillColor(outer_segment_color));
	drawable_list.push_back(DrawableEllipse(x_center, y_center, 700, 700, 0, 360));
	image.draw(drawable_list);
	drawable_list.clear();

	drawable_list.push_back(DrawableStrokeWidth(0));
	drawable_list.push_back(DrawableFillColor(inner_background_color));
	drawable_list.push_back(DrawableEllipse(x_center, y_center, 420, 420, 0, 360));
	image.draw(drawable_list);
	drawable_list.clear();
}

// Draw the encded ID into WhyCon marker
void draw_whycode_marker(int id, int id_bits)
{
	if( id_bits > 32 )
	{
		throw runtime_error("id_bits must be <= 32");
	}

	list<Drawable> drawable_list;
	list<Coordinate> coordinates_list;
	Image image = whycode_skeleton;
	
	// generate binary version of integer ID (lol stupid)
	string s = bitset<32>(id).to_string();
	if(verbose)
	{
		printf("Generating WhyCode Canvas for Id %d (encoding %d)\n", id, id);
		cout << "Converting ID = " << id << " to binary: " << s << endl;
	}

	drawable_list.push_back(DrawableStrokeWidth(0));
	// draw using the color for the ID polygons
	drawable_list.push_back(DrawableFillColor(polygon_color));

	// For each encoding bit
	double x1, y1, x2, y2;
	int bit_i;
	for(int i = 0; i < id_bits; i++)
	{
		// Calculate the pixel positions of each segment
	
		// get the i^{th} bit of the id, starting from the left of ID
		// the ID is stored all the way at the (right) end of a variable that is larger than itself
		bit_i = (id >> (id_bits - i - 1)) & 1;

		x1 = x_center + polygon_outer_radius * cos(-w * (2 * i + bit_i * 2.0) / 180.0 * M_PI);
		y1 = y_center + polygon_outer_radius * sin(-w * (2 * i + bit_i * 2.0) / 180.0 * M_PI);
		x2 = x_center + polygon_outer_radius * cos(-w * (2 * i + 1) / 180.0 * M_PI);
		y2 = y_center + polygon_outer_radius * sin(-w * (2 * i + 1) / 180.0 * M_PI);

		list<Coordinate> coordinates_list;
		coordinates_list.push_back(Coordinate(x_center, y_center));
		coordinates_list.push_back(Coordinate(x1, y1));
		coordinates_list.push_back(Coordinate(x2, y2));

		// Draw each of the segments onto the original WhyCon marker
		if(verbose)
		{
			printf("Drawing Segment Size: %f %f %f %f\n", x1 ,y1 ,x2, y2);
		}
		drawable_list.push_back(DrawablePolygon(coordinates_list));
		coordinates_list.clear();
	}
	image.draw(drawable_list);
	drawable_list.clear();

	// Draw a final white circle in the centre to complete the marker
	drawable_list.push_back(DrawableStrokeWidth(0));
	drawable_list.push_back(DrawableFillColor(inner_segment_color));
	drawable_list.push_back(DrawableEllipse(x_center, y_center, 240, 240, 0, 360));
	image.draw(drawable_list);
	drawable_list.clear();

	char infoText[32];
	sprintf(infoText, "%d-bit ID: %d", id_bits, id);
	drawable_list.push_back(DrawableText(1475, 130, infoText));
	drawable_list.push_back(DrawablePointSize(14));
	image.draw(drawable_list);
	drawable_list.clear();

	char fname[32];
	sprintf(fname, "%d_%08d.png", id_bits, id);
	printf("Rendering final image: %d (%d-bit) -> %s\n", id, id_bits, fname);
	image.magick("PNG");
	image.write(fname);
}

string decimal_to_binary(unsigned n){
        const int size = sizeof(n) * 8;
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

string manchester_encoding(string number)
{
        string result = "";

        for(int i = 0; i < number.size(); i ++)
        {
                if( number[i] == '0' )
                {
                        result += "01";
                }
                else if( number[i] == '1' )
                {
                        result += "10";
                }
                else
                {
                        throw runtime_error("Your number is not binary!");
                }
        }

        return result;
}

bool rotationally_self_asymmetric(string number)
{
        bool result = true;
        string rotated_number = number;

        int rotated_by = 0;
        while( (rotated_by < id_bits) && result)
        {
                rotate(rotated_number.begin(), rotated_number.begin() + 1, rotated_number.end());
//              cout << "\t" << number << " : " << rotated_number << endl;
                result = number != rotated_number;

                rotated_by ++;
        }

        return result;
}

bool is_duplicate(vector<string> existing_data, string new_data)
{
        bool result = false;

        int rotated_by = 0;
        while( (rotated_by < (2 * id_bits)) && (result == false) )
        {
                result = find(existing_data.begin(), existing_data.end(), new_data) != existing_data.end();
//              cout << new_data << " : " << result << endl;

                rotate(new_data.begin(), new_data.begin() + 1, new_data.end());
                rotated_by ++;
        }

        return result;
}

int main(int argc, char *argv[])
{
	if( argc == 1)
	{
		fprintf(stderr, "Not enough arguments.\n");
		display_help();
		return 1;
	}

	InitializeMagick(*argv);

	// generate the legacy WhyCon marker
	draw_whycon_marker(whycode_skeleton);

	id_bits = atoi(argv[1]);
	w = 360.0/(float)id_bits/2.0;
	int max_id  = pow(2, id_bits);
	n = pow(2, max_id);

        for(int id = 0; id < max_id; id ++)
        {
                string binarized_id = decimal_to_binary(id);

                // possibly add leading zeros to make all of the representations the same length
                binarized_id = string( id_bits - binarized_id.size(), '0').append( binarized_id );

//              cout << "Processing ID " << id << "->" << binarized_id << " with manchester encoding " << manchester_encoding(binarized_id) << endl;

                string manchester_representation = manchester_encoding(binarized_id);
                if( rotationally_self_asymmetric(manchester_representation) && (! is_duplicate( base_representations, manchester_representation) ) )
                {
                        base_representations.push_back( manchester_encoding(binarized_id) );
                        unique_ids.push_back(id);
                }
        }

	cout << unique_ids.size() << " unique markers in " << id_bits << "-bit family:" << endl;

	// generate WhyCode markers
//	for(int id = 1; id <= max_id; id ++)
	for(int id : unique_ids)
	{
		// draw the WhyCode marker with ID=id and id_bits bits in its ID
		draw_whycode_marker(id, id_bits);
	}

	return 0;
}
