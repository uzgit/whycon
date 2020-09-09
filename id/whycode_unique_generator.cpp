#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <bitset>
#include <Magick++.h>
#include "CNecklace.h"
#include <iostream>

using namespace Magick;
using namespace std;

// Define default parameters
bool verbose = true;
bool legacy = false;
bool single = false;
int hamming = 1;
int xc = 900;
int yc = 900;
int index = 0;
int n;
float w;

Image whycode_skeleton;
Image a4_grid_canvas;

void display_help()
{
    printf("\nUsage: whycon-id-gen <# id bits>\n");
}

// Draw the original WhyCon marker
void draw_whycon_marker(Image &image)
{
	list<Drawable> drawList;
	image.size("1800x1800");
	image.backgroundColor(Color("white"));
	image.erase();
	image.resolutionUnits(PixelsPerCentimeterResolution);
	image.density("300x300");

	// Generate original WhyCon marker
	drawList.push_back(DrawableStrokeColor("black"));
	drawList.push_back(DrawableFillColor("white"));
	drawList.push_back(DrawableEllipse(xc, yc, 899, 899, 0, 360));
	image.draw(drawList);
	drawList.clear();

	drawList.push_back(DrawableStrokeWidth(0));
	drawList.push_back(DrawableFillColor("black"));
	drawList.push_back(DrawableEllipse(xc, yc, 700, 700, 0, 360));
	image.draw(drawList);
	drawList.clear();

	drawList.push_back(DrawableStrokeWidth(0));
	drawList.push_back(DrawableFillColor("white"));
	drawList.push_back(DrawableEllipse(xc, yc, 420, 420, 0, 360));
	image.draw(drawList);
	drawList.clear();
}

// Draw the encded ID into WhyCon marker
void draw_whycode_marker(int id, int id_bits)
{
	if( id_bits > 32 )
	{
		throw runtime_error("id_bits must be <= 32");
	}

	list<Drawable> drawList;
	list<Coordinate> coordsList;
	Image image = whycode_skeleton;
	
	// generate binary version of integer ID (lol stupid)
	string s = bitset<32>(id).to_string();
	if(verbose)
	{
		printf("Generating WhyCode Canvas for Id %d (encoding %d)\n", id, id);
		cout << "Converting ID = " << id << " to binary: " << s << endl;
	}

	drawList.push_back(DrawableStrokeWidth(0));
	drawList.push_back(DrawableFillColor("black"));

	// For each encoding bit
	double x1, y1, x2, y2;
	int bit_i;
	for(int i = 0; i < id_bits; i++)
	{
		printf("i = %d, s.at(i + 32 - id_bits) = %c\n", i, s.at(i + 32 - id_bits));

		// Calculate the pixel positions of each segment
	
		// get the i^{th} bit of the id
		bit_i = (id >> (id_bits - i - 1)) & 1;
		printf("bit_i = %d\n", bit_i);

		x1 = xc + 650 * cos(-w * (2 * i + bit_i * 2.0) / 180.0 * M_PI);
		y1 = yc + 650 * sin(-w * (2 * i + bit_i * 2.0) / 180.0 * M_PI);
		x2 = xc + 650 * cos(-w * (2 * i + 1) / 180.0 * M_PI);
		y2 = yc + 650 * sin(-w * (2 * i + 1) / 180.0 * M_PI);

// orig		
/*
		x1 = xc + 650 * cos(-w * (2 * i + (s.at(i + 32 - id_bits) - '0') * 2.0) / 180.0 * M_PI);
		y1 = yc + 650 * sin(-w * (2 * i + (s.at(i + 32 - id_bits) - '0') * 2.0) / 180.0 * M_PI);
		x2 = xc + 650 * cos(-w * (2 * i + 1) / 180.0 * M_PI);
		y2 = yc + 650 * sin(-w * (2 * i + 1) / 180.0 * M_PI);
*/
		list<Coordinate> coordsList;
		coordsList.push_back(Coordinate(xc, yc));
		coordsList.push_back(Coordinate(x1, y1));
		coordsList.push_back(Coordinate(x2, y2));

		// Draw each of the segments onto the original WhyCon marker
		if(verbose)
		{
			printf("Drawing Segment Size: %f %f %f %f\n", x1 ,y1 ,x2, y2);
		}
		drawList.push_back(DrawablePolygon(coordsList));
		coordsList.clear();
	}
	image.draw(drawList);
	drawList.clear();

	// Draw a final white circle in the centre to complete the marker
	printf("Rendering final image: %d (encoding %d)  =>  %08d.png\n", id, id, id);
	drawList.push_back(DrawableStrokeWidth(0));
	drawList.push_back(DrawableFillColor("white"));
	drawList.push_back(DrawableEllipse(xc, yc, 240, 240, 0, 360));
	image.draw(drawList);
	drawList.clear();

	char infoText[32];
	sprintf(infoText, "%d-bit ID: %d", id_bits, id);
	drawList.push_back(DrawableText(1475, 130, infoText));
	drawList.push_back(DrawablePointSize(14));
	image.draw(drawList);
	drawList.clear();

	char fname[32];
	sprintf(fname, "%d_%08d.png", id_bits, id);
	image.magick("PNG");
	image.write(fname);
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

	int id_bits = atoi(argv[1]);
	w = 360.0/(float)id_bits/2.0;
	int max_id  = pow(2, id_bits);
	n = pow(2, max_id);

	// generate WhyCode markers
	for(int id = 1; id <= max_id; id ++)
	{
		// draw the WhyCode marker with ID=id and id_bits bits in its ID
		draw_whycode_marker(id, id_bits);
	}

	return 0;
}
