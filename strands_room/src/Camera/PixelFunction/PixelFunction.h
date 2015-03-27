#ifndef PixelFunction_H_
#define PixelFunction_H_
#include <vector>

class PixelFunction
{
	public:
	float r_mul;
	float g_mul;
	float b_mul;
	float d_mul;
	PixelFunction();
	~PixelFunction();
	
	void getValues(float * pixel);
	void update(std::vector<float*> data, float bias = 2, float random = 0, float step = 1);
	void addOutput(	std::vector<char *> & pixeldata_char,	std::vector<int> & pixeldata_counter);
	void load(char * buffer,int & pos);
};
#endif
