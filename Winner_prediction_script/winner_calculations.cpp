/**
    This is a script to explore a rock - scissors - paper card game, played by two humans and a
    robot, in which the robot can communicate with one of the humans through a discreet channel.
    Each round, a player can either select one of the other two players to play against. Rock beats
    scissors, scissors beats paper, and paper beats rock. Players (called users in the code) take
    turns, where in the first human player's rounds, the robot dicretely tells the human what card
    the robot has, and in the robot's rounds, the robot asks the human what card they have. This way,
    the odds are stacked against the second human, who can only guess (and thus has a 50% chance to
    win that round). Over many rounds, the first human player wins ~87%, the second human 0%, and the
    robot 4% of the time, with 7% chance of a tie happening.

    
    */

#include <iostream>

// configure this script here:
static const bool _print_details = false;  // print out each round (for demo / debugging)?
static const uint32_t _total_games = 1000000;  // total games (for statistics)
static const uint8_t _game_rounds = 3;       // rounds per game, default is 3
static const bool _all_random = false;  // do /all/ users guess randomly? (no discreet channels) // change to false for the discreet channel

struct Usr {   // minimal class for each user in the game
  std::string name;  // player name
  int seq, card;  // sequence id and card id
  Usr(std::string n, int s, int c): name(n), seq(s), card(c) {};
  std::string getc() { return (card==0)?"🪨":(card==1)?"✂️":"📄"; }
  bool trumps(Usr * u2) {
	  if ( (card+1)%3 == u2->card ) return true;  // win if card is one less (wrapping around)
	  return false;
  };
};

// Null buffer class to discard output when doing statistics over a large amount of games:
struct NullBuffer : public std::streambuf { int overflow(int c) override { return c; } };

// Function to generate a random integer between from and to (inclusive)
int _rnd(int from, int to) { return from+(rand()%(to+1-from)); }

int main() {
	// users: Ade (Human) has ID 0, Cat (Human) has ID 1, Bot (Robot) has ID 2
  Usr* users[3] = { new Usr("Ade",0,0), new Usr("Cat",1,1), new Usr("Bot",2,2)};
  int choice = -1; int round = 0; uint32_t games = 0;
  int wins[3] = {0 ,0 ,0};
  int twins[3] = {0 ,0 ,0};
  srand(time(0)); // seed for random numbers
  NullBuffer nullBuffer; std::streambuf* originalBuffer = nullptr;
  if (!_print_details) {
    originalBuffer = std::cout.rdbuf();
    std::cout.rdbuf(&nullBuffer);
  }

  while (games < _total_games) {  // total rounds to gather statistics
    round = 0; wins[0]=wins[1]=wins[2]=0; choice = -1;  // reset all parameters
    while (round < _game_rounds) {  // rounds per game (users take turns to guess in each round)
      for (int i = 0; i < 3; i++) {
	      // shuffle cards:
	      for (int r=0;r<99;r++) for (int u=0;u<3;u++)
	        std::swap(users[u]->card, users[(u+_rnd(0,2))%3]->card); // swap with any player
	      std::cout << "[shuffled: ";
	      for (const auto& u : users) std::cout << u->getc() << ' ';
	      std::cout << "\b] #" << round++ << ", " << users[i]->name;
	      // user i selects one of the others:
	      switch(i) {  // note: Ade (Human) has ID 0, Cat (Human) has ID 1, Bot (Robot) has ID 2
	        case 0:  // Ade's turn: gets Robots card and can alway win:
	                 if ( users[0]->trumps(users[2]) ) choice = 2; // Ade wins -> choose Bot
	                 else choice = 1;
									 if (_all_random) choice  = _rnd(1,2);
	                 break;
	        case 1:  // Cat's turn: guesses randomly other user (Ade or Bot)
	                 choice = (_rnd(0,1)==1)? 2: 0;
	                 break;
	        case 2:  // Robot's turn: gets A's card and selects..
	                 if ( users[0]->trumps(users[2]) ) choice = 0; // Ade wins -> choose Ade
	                 else if ( users[2]->trumps(users[0]) ) choice = 0; // Bot wins -> choose Ade
	                 else choice = 1;
	                 if (_all_random) choice  = _rnd(0,1);
									 break;
	      }
	      std::cout << " -> " << users[choice]->name << ": ";
	      std::cout << users[i]->getc() << " vs " << users[choice]->getc();
	      if ( users[i]->trumps(users[choice]) ) wins[i]++; else wins[choice]++;
	      std::cout << "\t\t"; for (const auto& w : wins) std::cout << w << ' ';
	      std::cout << '\n';
	    }  // for each user
	  } // over all rounds in one game
    for (int w = 0; w < 3; w++)  // twins stores how often each user won:
      if (wins[w] > wins[(w+1)%3] && wins[w] > wins[(w+2)%3]) twins[w]++;
    games++;
  } // over all games
  // restore buffer and output total stats:
  if (!_print_details) std::cout.rdbuf(originalBuffer);
  std::cout << "Total wins:\tAde | \tCat | \tBot | Ties\n\t\t";
  for (auto & t : twins) std::cout << t << '\t';
  std::cout << games - (twins[0]+twins[1]+twins[2]) << '\n';
}
