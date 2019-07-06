#ifndef RECTCLASSSCORE_H_
#define RECTCLASSSCORE_H_

#include <sstream>
#include <string>

namespace Yolo3
{
    enum YoloDetectorClasses//using coco for default cfg and weights
    {
    	Red,Green,Straight,Left,Right,
    };
}

template<typename _Tp> class RectClassScore
{
public:
	_Tp x, y, w, h;
	_Tp score;
	unsigned int class_type;
	bool enabled;

	inline std::string toString()
	{
		std::ostringstream out;
		out << class_type << "(x:" << x << ", y:" << y << ", w:" << w << ", h:" << h << ") =" << score;
		return out.str();
	}
	inline std::string GetClassString()
	{
		switch (class_type)
		{
			case Yolo3::Red: return "Red";
			case Yolo3::Green: return "Green";
			case Yolo3::Straight: return "Straight";
			case Yolo3::Left: return "Left";
			case Yolo3::Right: return "Right";
				
			default:return "error";
		
		
		}
	}

};

#endif /* RECTCLASSSCORE_H_ */
