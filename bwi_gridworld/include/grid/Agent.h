#ifndef bwi_gridworld_Agent_h__guard
#define bwi_gridworld_Agent_h__guard

#include <string>
#include <vector>

namespace bwi_gridworld {

	class Pos;

	class Agent {
	public:
		int x;
		int y;
		virtual char nextAction(const Pos &currentPos) = 0;
		virtual void eventFound(const Pos& currentPos) = 0;
		virtual Agent *clone(int id) = 0;

		virtual ~Agent(){};

	};


}
#endif
