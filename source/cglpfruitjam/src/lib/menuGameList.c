#include "cglp.h"

//working inccorectly requires mouse 
extern void addGamePakuPaku();
extern void addGameThunder();
extern void addGameBallTour();
extern void addGameHexmin();
extern void addGameSurvivor();
extern void addGamePinCLimb();
extern void addGameColorRoll();
extern void addGameCastN();
extern void addGameReflector();
extern void addGameFroooog();
extern void addGameRWheel();
extern void addGameLadderDrop();
extern void addGameGrapplingH();
extern void addGameInTow();
extern void addGameTimberTest();
  
  //new games
extern void addGameBCannon();
extern void addGameBaroll();
extern void addGameBamboo();
extern void addGameAerialBar();
extern void addGameBallsBombs();
extern void addGameAntLion();
extern void addGameTwofaced();
extern void addGameBombup();
extern void addGameBsfish();
extern void addGameBwalls();
extern void addGameCatapult();
extern void addGameCateP();
extern void addGameChargeBeam();
  //addGameCirclew();
extern void addGameCounterB();
extern void addGameCTower();
extern void addGameFracave();
extern void addGameDescents();
extern void addGameDivarr();
extern void addGameDLaser();

extern void  addGameBBlast();
extern void  addGameBalance();
extern void  addGameBreedc();
extern void  addGameCardq();
extern void  addGameCNodes();
extern void  addGameCrossLine();
extern void  addGameDarkCave();
extern void  addGameDFight();

void addGameSection(char* sectionName)
{
    Options o = { 0 };
    addGame(sectionName, "", NULL, 0, o, false, NULL);
}

void addGames() {
  addGameSection("==DEFAULT GAMES==");
  addGamePakuPaku();
  addGameThunder();
  addGameBallTour();
  addGameHexmin();
  addGameSurvivor();
  addGamePinCLimb();
  addGameColorRoll();
  addGameCastN();
  addGameReflector();
  addGameFroooog();
  addGameRWheel();
  addGameLadderDrop();
  addGameGrapplingH();
  addGameInTow();
  addGameTimberTest();
  
  //new games
  addGameSection("=======NEW=======");
  addGameBCannon();
  addGameBaroll();
  addGameBamboo();
  addGameAerialBar();
  addGameBallsBombs();
  addGameAntLion();
  addGameTwofaced();
  addGameBombup();
  addGameBsfish();
  addGameBwalls();
  addGameCatapult();
  addGameCateP();
  addGameChargeBeam();
  //addGameCirclew();
  addGameCounterB();
  addGameCTower();
  addGameFracave();
  addGameDescents();
  addGameDivarr();
  addGameDLaser();

  addGameSection("======MOUSE======");
  addGameBBlast();
  addGameBalance();
  addGameBreedc();
  addGameCardq();
  addGameCNodes();
  addGameCrossLine();
  addGameDarkCave();
  addGameDFight();

  //inccorect
  // addGameSection("======BUGGED=====");
  // addGameAccelB();

  // addGameSection("===BUGGED MOUSE==");

}
