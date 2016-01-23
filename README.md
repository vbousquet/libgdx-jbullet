# libgdx-jbullet
A fork of the Bullet physics library java port by Jezek2 to LibGdx library, to allow GWT support under Apache 2.0 license. 

This is a fork of [GBullet](https://code.google.com/p/gbullet/), a GWT port of [JBullet](http://jbullet.advel.cz/) which is a port of the [Bullet Physics engine](http://bulletphysics.org/wordpress/) using the [VecMath](https://java.net/projects/vecmath) library. All these libraries are under ZLib license except Vecmath which is GPL 2. That's the reason I made this fork ; it frees you from the vecmath dependency and use the math classes from LibGdx which is under Apache License 2.0, hence allowing commercia use.

## History

This is the first, pre-alpha release, largely under tested, without any support, likely full of bugs and such,... 
Think twice before using !

## Why would you use this library, and why would you not ?

This library was developped for the following reasons :
- optimize Android deployment size; the native Bullet port adds a few Mb to your APK. If you want to keep small, this version used with ProGuard will do wonders,
- enable HTML backend through GWT.

The counterparts are the following :
- absolutely no support; use at your own risk !
- the port lags behing the Bullet library; the port is somewhat at the level of the 2.72 version while the native port follows the latest Bullet version
- likely far slower than the native port.

A side benefit of this library is that it is full java so you don't have to go through the native memory management.

You can see it in action in [this published game](https://play.google.com/store/apps/details?id=org.softmotion.fpack.lite).

## Changes from the original port by Jezek2

- Added setAngularFactor et setLinearFactor in order to enable 2D use along 3D use (see [here](https://play.google.com/store/apps/details?id=org.softmotion.fpack.lite); dice use 3D physics, Air Hockey and Passe Trappe use 2D physics). 

## Usage

The work is too preliminary for a jar library. Just fork this repository and include the provided src folder in your build.

##License

Copyright 2015

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
