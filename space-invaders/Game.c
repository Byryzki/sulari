#include "Game.h"
#include <stdio.h>

volatile uint8_t shipcoords_ = 0;
volatile uint8_t aliencoords_ = 0;
volatile uint8_t aliendirection_ = 0;
volatile uint8_t bulletcoords_ = 0;
volatile uint8_t pointcount_  = 0;
volatile uint8_t bullety_ = 0;
volatile uint8_t ending_ = 0;

void GameSetup() {
	shipcoords_ = 3;
	SetPixel(shipcoords_+1, 6, 0, 0, 255);
	SetPixel(shipcoords_, 7, 0, 0, 255);
	SetPixel(shipcoords_+1, 7, 0, 0, 255);
	SetPixel(shipcoords_+2, 7, 0, 0, 255);

	ShipMove(0);

	aliencoords_ = 4;
	AlienMove();

	pointcount_ = 0;
	bulletcoords_ = 0;
	bullety_ = 0;

}

void ShipMove(uint8_t direction) {
	if ( ending_ == 1 ) {
		return;
	}
	if ( direction == 0 && shipcoords_ > 0 ) {

		// Clear right and top pixels:
		SetPixel(shipcoords_+1, 6, 0, 0, 0);
		SetPixel(shipcoords_+2, 7, 0, 0, 0);

		shipcoords_ -= 1;


		// Create left and new top pixels:
		SetPixel( shipcoords_+1, 6, 0, 0, 255);
		SetPixel(shipcoords_, 7, 0, 0, 255);

	}
	if (direction == 1 && shipcoords_ +3 < 8) {


		// Clear left and top pixels:
		SetPixel(shipcoords_+1, 6, 0, 0, 0);
		SetPixel(shipcoords_, 7, 0, 0, 0);

		shipcoords_  += 1;

		// Create right and new top pixels:
		SetPixel(shipcoords_+1, 6, 0, 0, 255);
		SetPixel(shipcoords_+2, 7, 0, 0, 255);
	}
}

void AlienMove(){
	if ( ending_ == 1 ) {
		return;
	}
	SetPixel(aliencoords_, 0, 0, 0, 0);
	if ( aliendirection_ == 1 ) {
		aliencoords_ += 1;
	}
	else {
		aliencoords_ -= 1;
	}

	SetPixel(aliencoords_ , 0, 0, 255, 0);

	if ( aliencoords_ == 0 ) {
		aliendirection_ = 1;
	}

	if ( aliencoords_ == 7 ) {
		aliendirection_ = 0;
	}
}

void BulletShoot() {
	if ( bulletcoords_ != 0 ) {
		return;
	} else {
		bulletcoords_ = shipcoords_+1;
		bullety_ = 5;
		SetPixel(bulletcoords_, bullety_, 255, 255, 0);
	}



}

void BulletMove() {
	if ( bulletcoords_ != 0 ) {
		SetPixel(bulletcoords_, bullety_, 0, 0, 0);
		bullety_ -= 1;
		SetPixel(bulletcoords_, bullety_, 255, 255, 0);
		if ( bullety_ == 0)
		BulletCheck();
	}


}

void BulletCheck() {
	if ( bullety_ == 0 ) {
		if ( bulletcoords_ == aliencoords_ ) {
				aliencoords_ = 4;
				pointcount_ += 1;
				if ( pointcount_ >= 7) {
					GameWon();
					return;
				}
				else {
					PointsUpdate();
					SetPixel(bulletcoords_, bullety_, 0, 0, 0);
					bulletcoords_ = 0;
					bullety_ = 0;
					return;
				}
		}
		usleep(800);
		SetPixel(bulletcoords_, bullety_, 0, 0, 0);
		bulletcoords_ = 0;
		bullety_ = 0;
	}
}

void PointsUpdate() {

	// Pisteet pikselein� oikeassa reunassa. Alhaalta yl�s. Mahtuu 6, joten 7 = GG
	SetPixel(7, 7 - pointcount_, 255, 0, 0);
}


void GameWon() {
	ending_ = 1;
	GameClear();

	// Draw YAY! with pixels
	SetPixel(0, 1, 255, 0, 0);
	SetPixel(1, 2, 255, 0, 0);
	SetPixel(1, 3, 255, 0, 0);
	SetPixel(1, 4, 255, 0, 0);
	SetPixel(2, 1, 255, 0, 0);

	SetPixel(2, 2, 0, 255, 0);
	SetPixel(2, 3, 0, 255, 0);
	SetPixel(2, 4, 0, 255, 0);
	SetPixel(3, 1, 0, 255, 0);
	SetPixel(3, 3, 0, 255, 0);
	SetPixel(4, 2, 0, 255, 0);
	SetPixel(4, 3, 0, 255, 0);
	SetPixel(4, 4, 0, 255, 0);

	SetPixel(4, 1, 0, 0, 255);
	SetPixel(5, 2, 0, 0, 255);
	SetPixel(5, 3, 0, 0, 255);
	SetPixel(5, 4, 0, 0, 255);
	SetPixel(6, 1, 0, 0, 255);

	SetPixel(7, 1, 255, 255, 0);
	SetPixel(7, 2, 255, 255, 0);
	SetPixel(7, 4, 255, 255, 0);




}

void GameRestart() {
	GameClear();
	ending_ = 0;
	GameSetup();

}

void Cheat() {
	if ( pointcount_ < 6 ) {
		pointcount_ += 1;
		PointsUpdate();
	}
}

void GameClear() {
	for (uint8_t x = 0; x < 8; x++) {
		for (uint8_t y = 0; y < 8; y++ ) {
			SetPixel(x, y, 0, 0, 0);
		}
	}
}




