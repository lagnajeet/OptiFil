#include <ArduinoQueue.h>
#include <SimpleCLI.h>
#include <CommandHandler.h>
#include <ArduinoTimer.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESP8266SSDP.h>
#include <ESP8266mDNS.h>        // Include the mDNS library

#include <SPI.h>

#include <Adafruit_NeoPixel.h>

#include <avr/pgmspace.h>
#include <FS.h>
const int sensorPin = A0;  // ESP8266 Analog Pin ADC0 = A0

// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Snap_Angle                           0x42
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

byte initComplete=0;
byte testctr=0;
unsigned long currTime;
unsigned long timer;
unsigned long pollTimer;
unsigned long debounceTime=0;
volatile int xydat[2];
volatile int Squal;
volatile byte movementflag=0;
const int ncs = 15;   //this is the slave select pin
const int button1 = 5;		//External Input
const int button0 = 0;		//Enable disbale button
const int jamLed = 4;
const int runoutLed = 2;    //This is inverted
const int notifyPin = 16;   //this is active low in CR-10 i.e. low means ok and high means problem
const unsigned short firmware_length = 3070;

int zeroSpeedTh=5;			// Line 1 , Specifies which line in the settings file has value for this variable
int zeroSpeedTime=10;		// Line 2
int minSqVal = 5;			// Line 3
int LEDBrightness = 16;		// Line 4 this number decides the brightnes of the indicator LED. 0 means disabled and any other number means that much brighness
//int ExternalInput = 0;		//Line 5 if enabled it listens to external signal for turning sensing on.
int JamDetection = 0;		//Line 5 Enable Jam detection if not set to 0
int RunOutDetection = 1;	//Line 6 Enable runout detection if set to 1 

uint8_t CurrentSensorValue = 0;		//Hall effect current sensorvalue
int incomingByte = 0;
#define QUEUE_SIZE_ITEMS 512
uint8_t currentQueue[QUEUE_SIZE_ITEMS] = { 0 };
int currentSensnorCount = 0;
long xDist = 0;
long yDist = 0;
bool tempLigh = true;
long rdist = 0;
long rspeed = 0;
int rspeedBelowTh = 0;
int rspeedCheckCount = 0;
long temprdist = 0;
int distCount = 0;
int samplingPeriod = 10;  //in ms
bool jammed = false;
bool runout = false;
bool startMonitoring = true;
int globalR, globalG, globalB;
bool prevButton0State = true;
int longpressCounter = 0;

SPISettings ADNS(1500000, MSBFIRST, SPI_MODE3);
#define numLEDS 2
#define MAXDIFFERENCE 25
#define MAXVARIANCE 15
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numLEDS, jamLed, NEO_GRB + NEO_KHZ800);
static const char ispinJs[] PROGMEM = "!function(t,e){\"object\"==typeof exports&&\"undefined\"!=typeof module?module.exports=e():\"function\"==typeof define&&define.amd?define(e):t.ISpin=e()}(this,function(){\"use strict\";function t(t,e){for(var n=0;n<e.length;n++){var s=e[n];s.enumerable=s.enumerable||!1,s.configurable=!0,\"value\"in s&&(s.writable=!0),Object.defineProperty(t,s.key,s)}}function e(t,e,n){return e in t?Object.defineProperty(t,e,{value:n,enumerable:!0,configurable:!0,writable:!0}):t[e]=n,t}function n(){return(n=Object.assign||function(t){for(var e=1;e<arguments.length;e++){var n=arguments[e];for(var s in n)Object.prototype.hasOwnProperty.call(n,s)&&(t[s]=n[s])}return t}).apply(this,arguments)}function s(t){return function(t){if(Array.isArray(t)){for(var e=0,n=new Array(t.length);e<t.length;e++)n[e]=t[e];return n}}(t)||function(t){if(Symbol.iterator in Object(t)||\"[object Arguments]\"===Object.prototype.toString.call(t))return Array.from(t)}(t)||function(){throw new TypeError(\"Invalid attempt to spread non-iterable instance\")}()}var i=function(){function i(t,e){!function(t,e){if(!(t instanceof e))throw new TypeError(\"Cannot call a class as a function\")}(this,i),this.el=t,this.options={},this._onKeyDown=this._onKeyDown.bind(this),this._onMouseDown=this._onMouseDown.bind(this),this._onMouseUp=this._onMouseUp.bind(this),this._onMouseLeave=this._onMouseLeave.bind(this),this._onWheel=this._onWheel.bind(this),this.build(),this.update(e)}var r,a,p;return r=i,(a=[{key:\"build\",value:function(){var t=this;this._wrapper=document.createElement(\"div\"),this.el.parentNode&&this.el.parentNode.replaceChild(this._wrapper,this.el),this._wrapper.appendChild(this.el),this._buttons={inc:document.createElement(\"button\"),dec:document.createElement(\"button\")},Object.keys(this._buttons).forEach(function(e){var n=t._buttons[e];t._wrapper.appendChild(n),n.setAttribute(\"type\",\"button\"),n.addEventListener(\"mousedown\",t._onMouseDown),n.addEventListener(\"mouseup\",t._onMouseUp),n.addEventListener(\"mouseleave\",t._onMouseLeave)}),this.el.addEventListener(\"keydown\",this._onKeyDown),this.el.addEventListener(\"wheel\",this._onWheel)}},{key:\"update\",value:function(t){var s=this;(t=function(t){for(var n=1;n<arguments.length;n++){var s=null!=arguments[n]?arguments[n]:{},i=Object.keys(s);\"function\"==typeof Object.getOwnPropertySymbols&&(i=i.concat(Object.getOwnPropertySymbols(s).filter(function(t){return Object.getOwnPropertyDescriptor(s,t).enumerable}))),i.forEach(function(n){e(t,n,s[n])})}return t}({},i.DEFAULTS,this.options,t)).wrapperClass!==this.options.wrapperClass&&(this.options.wrapperClass&&this._wrapper.classList.remove(this.options.wrapperClass),t.wrapperClass&&this._wrapper.classList.add(t.wrapperClass)),t.buttonsClass!==this.options.buttonsClass&&(this.options.buttonsClass&&Object.keys(this._buttons).forEach(function(t){s._buttons[t].classList.remove(s.options.buttonsClass),s._buttons[t].classList.remove(s.options.buttonsClass+\"-\"+t)}),t.buttonsClass&&Object.keys(this._buttons).forEach(function(e){s._buttons[e].classList.add(t.buttonsClass),s._buttons[e].classList.add(t.buttonsClass+\"-\"+e)})),this.disabled=t.disabled,n(this.options,t)}},{key:\"destroy\",value:function(){this._wrapper.parentNode&&this._wrapper.parentNode.replaceChild(this.el,this._wrapper),delete this.el,delete this._wrapper,delete this._buttons}},{key:\"_onKeyDown\",value:function(t){switch(t.keyCode){case 38:return t.preventDefault(),this.spin(this.options.step);case 40:return t.preventDefault(),this.spin(-this.options.step);case 33:return t.preventDefault(),this.spin(this.options.pageStep);case 34:return t.preventDefault(),this.spin(-this.options.pageStep)}}},{key:\"_onMouseDown\",value:function(t){t.preventDefault();var e=t.currentTarget===this._buttons.inc?1:-1;this.spin(e*this.options.step),this.el.focus(),this._startSpinning(e)}},{key:\"_onMouseUp\",value:function(t){this._stopSpinning()}},{key:\"_onMouseLeave\",value:function(t){this._stopSpinning()}},{key:\"_startSpinning\",value:function(t){var e=this;this._stopSpinning(),this._spinTimer=setInterval(function(){return e.spin(t*e.options.step)},this.options.repeatInterval)}},{key:\"_stopSpinning\",value:function(){clearInterval(this._spinTimer)}},{key:\"_onWheel\",value:function(t){if(document.activeElement===this.el){t.preventDefault();var e=t.deltaY>0?-1:1;this.spin(e*this.options.step)}}},{key:\"adjustValue\",value:function(t){return t=Number(t.toFixed(this.precision)),null!=this.options.max&&t>this.options.max&&(t=this.options.max),null!=this.options.min&&t<this.options.min&&(t=this.options.min),t}},{key:\"wrapValue\",value:function(t){return this.options.wrapOverflow&&null!=this.options.max&&null!=this.options.min&&(t<this.options.min?t=this.options.max:t>this.options.max&&(t=this.options.min)),t}},{key:\"spin\",value:function(t){this.value=this.adjustValue(this.wrapValue(this.value+t))}},{key:\"value\",get:function(){return this.options.parse(this.el.value)||0},set:function(t){var e=this.options.format(this.options.parse(String(t)));this.el.value=e,this.options.onChange&&this.options.onChange(e)}},{key:\"disabled\",get:function(){return this._buttons.inc.disabled},set:function(t){this.disabled!==t&&(this._buttons.inc.disabled=this._buttons.dec.disabled=t)}},{key:\"precision\",get:function(){return Math.max.apply(Math,s([this.options.step,this.options.min].filter(function(t){return null!=t}).map(o)))}}])&&t(r.prototype,a),p&&t(r,p),i}();function o(t){return(String(t).split(\".\")[1]||\"\").length}return i.DEFAULTS={wrapperClass:\"ispin-wrapper\",buttonsClass:\"ispin-button\",step:1,pageStep:10,disabled:!1,repeatInterval:200,wrapOverflow:!1,parse:Number,format:String},i});";
static const char ispinCss[] PROGMEM = ".ispin-wrapper{position:relative;display:inline-block;overflow:hidden;padding:0;vertical-align:middle}.ispin-wrapper .ispin-button{position:absolute;display:block;height:60%;min-height:0;min-width:0;width:18px;padding:0;margin:0;right:0;border:none;background:0 0;cursor:pointer}.ispin-wrapper .ispin-button:before{content:'';display:inline-block;-webkit-transform:translateY(-50%);transform:translateY(-50%);border-left:5px solid transparent;border-right:5px solid transparent}.ispin-wrapper .ispin-button.ispin-button-inc{top:0}.ispin-wrapper .ispin-button.ispin-button-inc:before{border-bottom:5px solid #000}.ispin-wrapper .ispin-button.ispin-button-inc:disabled:before{border-bottom-color:grey}.ispin-wrapper .ispin-button.ispin-button-dec{bottom:0}.ispin-wrapper .ispin-button.ispin-button-dec:before{border-top:5px solid #000}.ispin-wrapper .ispin-button.ispin-button-dec:disabled:before{border-top-color:grey}";
static const char gaugeJs[] PROGMEM = "(function(){var t,i,e,s,n,o,a,h,r,l,p,c,u,d=[].slice,g={}.hasOwnProperty,m=function(t,i){for(var e in i)g.call(i,e)&&(t[e]=i[e]);function s(){this.constructor=t}return s.prototype=i.prototype,t.prototype=new s,t.__super__=i.prototype,t};!function(){var t,i,e,s,n,o,a;for(e=0,n=(a=[\"ms\",\"moz\",\"webkit\",\"o\"]).length;e<n&&(o=a[e],!window.requestAnimationFrame);e++)window.requestAnimationFrame=window[o+\"RequestAnimationFrame\"],window.cancelAnimationFrame=window[o+\"CancelAnimationFrame\"]||window[o+\"CancelRequestAnimationFrame\"];t=null,s=0,i={},requestAnimationFrame?window.cancelAnimationFrame||(t=window.requestAnimationFrame,window.requestAnimationFrame=function(e,n){var o;return o=++s,t(function(){if(!i[o])return e()},n),o},window.cancelAnimationFrame=function(t){return i[t]=!0}):(window.requestAnimationFrame=function(t,i){var e,s,n,o;return e=(new Date).getTime(),o=Math.max(0,16-(e-n)),s=window.setTimeout(function(){return t(e+o)},o),n=e+o,s},window.cancelAnimationFrame=function(t){return clearTimeout(t)})}(),u=function(t){var i,e;for(t-=3600*(i=Math.floor(t/3600))+60*(e=Math.floor((t-3600*i)/60)),t+=\"\",e+=\"\";e.length<2;)e=\"0\"+e;for(;t.length<2;)t=\"0\"+t;return(i=i?i+\":\":\"\")+e+\":\"+t},p=function(){var t,i,e;return e=(i=1<=arguments.length?d.call(arguments,0):[])[0],t=i[1],r(e.toFixed(t))},c=function(t,i){var e,s,n;s={};for(e in t)g.call(t,e)&&(n=t[e],s[e]=n);for(e in i)g.call(i,e)&&(n=i[e],s[e]=n);return s},r=function(t){var i,e,s,n;for(s=(e=(t+=\"\").split(\".\"))[0],n=\"\",e.length>1&&(n=\".\"+e[1]),i=/(\\d+)(\\d{3})/;i.test(s);)s=s.replace(i,\"$1,$2\");return s+n},l=function(t){return\"#\"===t.charAt(0)?t.substring(1,7):t},h=function(){function t(t,i){null==t&&(t=!0),this.clear=null==i||i,t&&AnimationUpdater.add(this)}return t.prototype.animationSpeed=32,t.prototype.update=function(t){var i;return null==t&&(t=!1),!(!t&&this.displayedValue===this.value)&&(this.ctx&&this.clear&&this.ctx.clearRect(0,0,this.canvas.width,this.canvas.height),i=this.value-this.displayedValue,Math.abs(i/this.animationSpeed)<=.001?this.displayedValue=this.value:this.displayedValue=this.displayedValue+i/this.animationSpeed,this.render(),!0)},t}(),e=function(t){function i(){return i.__super__.constructor.apply(this,arguments)}return m(i,h),i.prototype.displayScale=1,i.prototype.forceUpdate=!0,i.prototype.setTextField=function(t,i){return this.textField=t instanceof a?t:new a(t,i)},i.prototype.setMinValue=function(t,i){var e,s,n,o,a;if(this.minValue=t,null==i&&(i=!0),i){for(this.displayedValue=this.minValue,a=[],s=0,n=(o=this.gp||[]).length;s<n;s++)e=o[s],a.push(e.displayedValue=this.minValue);return a}},i.prototype.setOptions=function(t){return null==t&&(t=null),this.options=c(this.options,t),this.textField&&(this.textField.el.style.fontSize=t.fontSize+\"px\"),this.options.angle>.5&&(this.options.angle=.5),this.configDisplayScale(),this},i.prototype.configDisplayScale=function(){var t,i,e,s,n;return s=this.displayScale,!1===this.options.highDpiSupport?delete this.displayScale:(i=window.devicePixelRatio||1,t=this.ctx.webkitBackingStorePixelRatio||this.ctx.mozBackingStorePixelRatio||this.ctx.msBackingStorePixelRatio||this.ctx.oBackingStorePixelRatio||this.ctx.backingStorePixelRatio||1,this.displayScale=i/t),this.displayScale!==s&&(n=this.canvas.G__width||this.canvas.width,e=this.canvas.G__height||this.canvas.height,this.canvas.width=n*this.displayScale,this.canvas.height=e*this.displayScale,this.canvas.style.width=n+\"px\",this.canvas.style.height=e+\"px\",this.canvas.G__width=n,this.canvas.G__height=e),this},i.prototype.parseValue=function(t){return t=parseFloat(t)||Number(t),isFinite(t)?t:0},i}(),a=function(){function t(t,i){this.el=t,this.fractionDigits=i}return t.prototype.render=function(t){return this.el.innerHTML=p(t.displayedValue,this.fractionDigits)},t}(),t=function(t){function i(t,e){if(this.elem=t,this.text=null!=e&&e,i.__super__.constructor.call(this),void 0===this.elem)throw new Error(\"The element isn't defined.\");this.value=1*this.elem.innerHTML,this.text&&(this.value=0)}return m(i,h),i.prototype.displayedValue=0,i.prototype.value=0,i.prototype.setVal=function(t){return this.value=1*t},i.prototype.render=function(){var t;return t=this.text?u(this.displayedValue.toFixed(0)):r(p(this.displayedValue)),this.elem.innerHTML=t},i}(),o=function(t){function i(t){if(this.gauge=t,void 0===this.gauge)throw new Error(\"The element isn't defined.\");this.ctx=this.gauge.ctx,this.canvas=this.gauge.canvas,i.__super__.constructor.call(this,!1,!1),this.setOptions()}return m(i,h),i.prototype.displayedValue=0,i.prototype.value=0,i.prototype.options={strokeWidth:.035,length:.1,color:\"#000000\",iconPath:null,iconScale:1,iconAngle:0},i.prototype.img=null,i.prototype.setOptions=function(t){if(null==t&&(t=null),this.options=c(this.options,t),this.length=2*this.gauge.radius*this.gauge.options.radiusScale*this.options.length,this.strokeWidth=this.canvas.height*this.options.strokeWidth,this.maxValue=this.gauge.maxValue,this.minValue=this.gauge.minValue,this.animationSpeed=this.gauge.animationSpeed,this.options.angle=this.gauge.options.angle,this.options.iconPath)return this.img=new Image,this.img.src=this.options.iconPath},i.prototype.render=function(){var t,i,e,s,n,o,a,h,r;if(t=this.gauge.getAngle.call(this,this.displayedValue),h=Math.round(this.length*Math.cos(t)),r=Math.round(this.length*Math.sin(t)),o=Math.round(this.strokeWidth*Math.cos(t-Math.PI/2)),a=Math.round(this.strokeWidth*Math.sin(t-Math.PI/2)),i=Math.round(this.strokeWidth*Math.cos(t+Math.PI/2)),e=Math.round(this.strokeWidth*Math.sin(t+Math.PI/2)),this.ctx.beginPath(),this.ctx.fillStyle=this.options.color,this.ctx.arc(0,0,this.strokeWidth,0,2*Math.PI,!1),this.ctx.fill(),this.ctx.beginPath(),this.ctx.moveTo(o,a),this.ctx.lineTo(h,r),this.ctx.lineTo(i,e),this.ctx.fill(),this.img)return s=Math.round(this.img.width*this.options.iconScale),n=Math.round(this.img.height*this.options.iconScale),this.ctx.save(),this.ctx.translate(h,r),this.ctx.rotate(t+Math.PI/180*(90+this.options.iconAngle)),this.ctx.drawImage(this.img,-s/2,-n/2,s,n),this.ctx.restore()},i}(),function(){function t(t){this.elem=t}t.prototype.updateValues=function(t){return this.value=t[0],this.maxValue=t[1],this.avgValue=t[2],this.render()},t.prototype.render=function(){var t,i;return this.textField&&this.textField.text(p(this.value)),0===this.maxValue&&(this.maxValue=2*this.avgValue),i=this.value/this.maxValue*100,t=this.avgValue/this.maxValue*100,$(\".bar-value\",this.elem).css({width:i+\"%\"}),$(\".typical-value\",this.elem).css({width:t+\"%\"})}}(),n=function(t){function i(t){var e,s;this.canvas=t,i.__super__.constructor.call(this),this.percentColors=null,\"undefined\"!=typeof G_vmlCanvasManager&&(this.canvas=window.G_vmlCanvasManager.initElement(this.canvas)),this.ctx=this.canvas.getContext(\"2d\"),e=this.canvas.clientHeight,s=this.canvas.clientWidth,this.canvas.height=e,this.canvas.width=s,this.gp=[new o(this)],this.setOptions()}return m(i,e),i.prototype.elem=null,i.prototype.value=[20],i.prototype.maxValue=80,i.prototype.minValue=0,i.prototype.displayedAngle=0,i.prototype.displayedValue=0,i.prototype.lineWidth=40,i.prototype.paddingTop=.1,i.prototype.paddingBottom=.1,i.prototype.percentColors=null,i.prototype.options={colorStart:\"#6fadcf\",colorStop:void 0,gradientType:0,strokeColor:\"#e0e0e0\",pointer:{length:.8,strokeWidth:.035,iconScale:1},angle:.15,lineWidth:.44,radiusScale:1,fontSize:40,limitMax:!1,limitMin:!1},i.prototype.setOptions=function(t){var e,s,n,o,a;for(null==t&&(t=null),i.__super__.setOptions.call(this,t),this.configPercentColors(),this.extraPadding=0,this.options.angle<0&&(o=Math.PI*(1+this.options.angle),this.extraPadding=Math.sin(o)),this.availableHeight=this.canvas.height*(1-this.paddingTop-this.paddingBottom),this.lineWidth=this.availableHeight*this.options.lineWidth,this.radius=(this.availableHeight-this.lineWidth/2)/(1+this.extraPadding),this.ctx.clearRect(0,0,this.canvas.width,this.canvas.height),s=0,n=(a=this.gp).length;s<n;s++)(e=a[s]).setOptions(this.options.pointer),e.render();return this.render(),this},i.prototype.configPercentColors=function(){var t,i,e,s,n,o,a;if(this.percentColors=null,void 0!==this.options.percentColors){for(this.percentColors=new Array,o=[],e=s=0,n=this.options.percentColors.length-1;0<=n?s<=n:s>=n;e=0<=n?++s:--s)a=parseInt(l(this.options.percentColors[e][1]).substring(0,2),16),i=parseInt(l(this.options.percentColors[e][1]).substring(2,4),16),t=parseInt(l(this.options.percentColors[e][1]).substring(4,6),16),o.push(this.percentColors[e]={pct:this.options.percentColors[e][0],color:{r:a,g:i,b:t}});return o}},i.prototype.set=function(t){var i,e,s,n,a,h,r,l,p;for(t instanceof Array||(t=[t]),e=s=0,r=t.length-1;0<=r?s<=r:s>=r;e=0<=r?++s:--s)t[e]=this.parseValue(t[e]);if(t.length>this.gp.length)for(e=n=0,l=t.length-this.gp.length;0<=l?n<l:n>l;e=0<=l?++n:--n)(i=new o(this)).setOptions(this.options.pointer),this.gp.push(i);else t.length<this.gp.length&&(this.gp=this.gp.slice(this.gp.length-t.length));for(e=0,a=0,h=t.length;a<h;a++)(p=t[a])>this.maxValue?this.options.limitMax?p=this.maxValue:this.maxValue=p+1:p<this.minValue&&(this.options.limitMin?p=this.minValue:this.minValue=p-1),this.gp[e].value=p,this.gp[e++].setOptions({minValue:this.minValue,maxValue:this.maxValue,angle:this.options.angle});return this.value=Math.max(Math.min(t[t.length-1],this.maxValue),this.minValue),AnimationUpdater.run(this.forceUpdate),this.forceUpdate=!1},i.prototype.getAngle=function(t){return(1+this.options.angle)*Math.PI+(t-this.minValue)/(this.maxValue-this.minValue)*(1-2*this.options.angle)*Math.PI},i.prototype.getColorForPercentage=function(t,i){var e,s,n,o,a,h,r;if(0===t)e=this.percentColors[0].color;else for(e=this.percentColors[this.percentColors.length-1].color,n=o=0,h=this.percentColors.length-1;0<=h?o<=h:o>=h;n=0<=h?++o:--o)if(t<=this.percentColors[n].pct){!0===i?(r=this.percentColors[n-1]||this.percentColors[0],s=this.percentColors[n],a=(t-r.pct)/(s.pct-r.pct),e={r:Math.floor(r.color.r*(1-a)+s.color.r*a),g:Math.floor(r.color.g*(1-a)+s.color.g*a),b:Math.floor(r.color.b*(1-a)+s.color.b*a)}):e=this.percentColors[n].color;break}return\"rgb(\"+[e.r,e.g,e.b].join(\",\")+\")\"},i.prototype.getColorForValue=function(t,i){var e;return e=(t-this.minValue)/(this.maxValue-this.minValue),this.getColorForPercentage(e,i)},i.prototype.renderStaticLabels=function(t,i,e,s){var n,o,a,h,r,l,c,u,d,g;for(this.ctx.save(),this.ctx.translate(i,e),l=/\\d+\\.?\\d?/,r=(n=t.font||\"10px Times\").match(l)[0],u=n.slice(r.length),o=parseFloat(r)*this.displayScale,this.ctx.font=o+u,this.ctx.fillStyle=t.color||\"#000000\",this.ctx.textBaseline=\"bottom\",this.ctx.textAlign=\"center\",a=0,h=(c=t.labels).length;a<h;a++)void 0!==(g=c[a]).label?(!this.options.limitMin||g>=this.minValue)&&(!this.options.limitMax||g<=this.maxValue)&&(r=(n=g.font||t.font).match(l)[0],u=n.slice(r.length),o=parseFloat(r)*this.displayScale,this.ctx.font=o+u,d=this.getAngle(g.label)-3*Math.PI/2,this.ctx.rotate(d),this.ctx.fillText(p(g.label,t.fractionDigits),0,-s-this.lineWidth/2),this.ctx.rotate(-d)):(!this.options.limitMin||g>=this.minValue)&&(!this.options.limitMax||g<=this.maxValue)&&(d=this.getAngle(g)-3*Math.PI/2,this.ctx.rotate(d),this.ctx.fillText(p(g,t.fractionDigits),0,-s-this.lineWidth/2),this.ctx.rotate(-d));return this.ctx.restore()},i.prototype.renderTicks=function(t,i,e,s){var n,o,a,h,r,l,p,c,u,d,g,m,x,f,v,y,V,w,S,M;if(t!=={}){for(l=t.divisions||0,w=t.subDivisions||0,a=t.divColor||\"#fff\",f=t.subColor||\"#fff\",h=t.divLength||.7,y=t.subLength||.2,u=parseFloat(this.maxValue)-parseFloat(this.minValue),d=parseFloat(u)/parseFloat(t.divisions),v=parseFloat(d)/parseFloat(t.subDivisions),n=parseFloat(this.minValue),o=0+v,r=(c=u/400)*(t.divWidth||1),V=c*(t.subWidth||1),m=[],S=p=0,g=l+1;p<g;S=p+=1)this.ctx.lineWidth=this.lineWidth*h,x=this.lineWidth/2*(1-h),M=this.radius*this.options.radiusScale+x,this.ctx.strokeStyle=a,this.ctx.beginPath(),this.ctx.arc(0,0,M,this.getAngle(n-r),this.getAngle(n+r),!1),this.ctx.stroke(),o=n+v,n+=d,S!==t.divisions&&w>0?m.push(function(){var t,i,e;for(e=[],t=0,i=w-1;t<i;t+=1)this.ctx.lineWidth=this.lineWidth*y,x=this.lineWidth/2*(1-y),M=this.radius*this.options.radiusScale+x,this.ctx.strokeStyle=f,this.ctx.beginPath(),this.ctx.arc(0,0,M,this.getAngle(o-V),this.getAngle(o+V),!1),this.ctx.stroke(),e.push(o+=v);return e}.call(this)):m.push(void 0);return m}},i.prototype.render=function(){var t,i,e,s,n,o,a,h,r,l,p,c,u,d,g,m;if(g=this.canvas.width/2,e=this.canvas.height*this.paddingTop+this.availableHeight-(this.radius+this.lineWidth/2)*this.extraPadding,t=this.getAngle(this.displayedValue),this.textField&&this.textField.render(this),this.ctx.lineCap=\"butt\",l=this.radius*this.options.radiusScale,this.options.staticLabels&&this.renderStaticLabels(this.options.staticLabels,g,e,l),this.options.staticZones)for(this.ctx.save(),this.ctx.translate(g,e),this.ctx.lineWidth=this.lineWidth,s=0,o=(p=this.options.staticZones).length;s<o;s++)r=(m=p[s]).min,this.options.limitMin&&r<this.minValue&&(r=this.minValue),h=m.max,this.options.limitMax&&h>this.maxValue&&(h=this.maxValue),d=this.radius*this.options.radiusScale,m.height&&(this.ctx.lineWidth=this.lineWidth*m.height,u=this.lineWidth/2*(m.offset||1-m.height),d=this.radius*this.options.radiusScale+u),this.ctx.strokeStyle=m.strokeStyle,this.ctx.beginPath(),this.ctx.arc(0,0,d,this.getAngle(r),this.getAngle(h),!1),this.ctx.stroke();else void 0!==this.options.customFillStyle?i=this.options.customFillStyle(this):null!==this.percentColors?i=this.getColorForValue(this.displayedValue,this.options.generateGradient):void 0!==this.options.colorStop?((i=0===this.options.gradientType?this.ctx.createRadialGradient(g,e,9,g,e,70):this.ctx.createLinearGradient(0,0,g,0)).addColorStop(0,this.options.colorStart),i.addColorStop(1,this.options.colorStop)):i=this.options.colorStart,this.ctx.strokeStyle=i,this.ctx.beginPath(),this.ctx.arc(g,e,l,(1+this.options.angle)*Math.PI,t,!1),this.ctx.lineWidth=this.lineWidth,this.ctx.stroke(),this.ctx.strokeStyle=this.options.strokeColor,this.ctx.beginPath(),this.ctx.arc(g,e,l,t,(2-this.options.angle)*Math.PI,!1),this.ctx.stroke(),this.ctx.save(),this.ctx.translate(g,e);for(this.options.renderTicks&&this.renderTicks(this.options.renderTicks,g,e,l),this.ctx.restore(),this.ctx.translate(g,e),n=0,a=(c=this.gp).length;n<a;n++)c[n].update(!0);return this.ctx.translate(-g,-e)},i}(),i=function(t){function i(t){this.canvas=t,i.__super__.constructor.call(this),\"undefined\"!=typeof G_vmlCanvasManager&&(this.canvas=window.G_vmlCanvasManager.initElement(this.canvas)),this.ctx=this.canvas.getContext(\"2d\"),this.setOptions(),this.render()}return m(i,e),i.prototype.lineWidth=15,i.prototype.displayedValue=0,i.prototype.value=33,i.prototype.maxValue=80,i.prototype.minValue=0,i.prototype.options={lineWidth:.1,colorStart:\"#6f6ea0\",colorStop:\"#c0c0db\",strokeColor:\"#eeeeee\",shadowColor:\"#d5d5d5\",angle:.35,radiusScale:1},i.prototype.getAngle=function(t){return(1-this.options.angle)*Math.PI+(t-this.minValue)/(this.maxValue-this.minValue)*(2+this.options.angle-(1-this.options.angle))*Math.PI},i.prototype.setOptions=function(t){return null==t&&(t=null),i.__super__.setOptions.call(this,t),this.lineWidth=this.canvas.height*this.options.lineWidth,this.radius=this.options.radiusScale*(this.canvas.height/2-this.lineWidth/2),this},i.prototype.set=function(t){return this.value=this.parseValue(t),this.value>this.maxValue?this.options.limitMax?this.value=this.maxValue:this.maxValue=this.value:this.value<this.minValue&&(this.options.limitMin?this.value=this.minValue:this.minValue=this.value),AnimationUpdater.run(this.forceUpdate),this.forceUpdate=!1},i.prototype.render=function(){var t,i,e,s;return t=this.getAngle(this.displayedValue),s=this.canvas.width/2,e=this.canvas.height/2,this.textField&&this.textField.render(this),(i=this.ctx.createRadialGradient(s,e,39,s,e,70)).addColorStop(0,this.options.colorStart),i.addColorStop(1,this.options.colorStop),this.radius-this.lineWidth/2,this.radius+this.lineWidth/2,this.ctx.strokeStyle=this.options.strokeColor,this.ctx.beginPath(),this.ctx.arc(s,e,this.radius,(1-this.options.angle)*Math.PI,(2+this.options.angle)*Math.PI,!1),this.ctx.lineWidth=this.lineWidth,this.ctx.lineCap=\"round\",this.ctx.stroke(),this.ctx.strokeStyle=i,this.ctx.beginPath(),this.ctx.arc(s,e,this.radius,(1-this.options.angle)*Math.PI,t,!1),this.ctx.stroke()},i}(),s=function(t){function e(){return e.__super__.constructor.apply(this,arguments)}return m(e,i),e.prototype.strokeGradient=function(t,i,e,s){var n;return(n=this.ctx.createRadialGradient(t,i,e,t,i,s)).addColorStop(0,this.options.shadowColor),n.addColorStop(.12,this.options._orgStrokeColor),n.addColorStop(.88,this.options._orgStrokeColor),n.addColorStop(1,this.options.shadowColor),n},e.prototype.setOptions=function(t){var i,s,n,o;return null==t&&(t=null),e.__super__.setOptions.call(this,t),o=this.canvas.width/2,i=this.canvas.height/2,s=this.radius-this.lineWidth/2,n=this.radius+this.lineWidth/2,this.options._orgStrokeColor=this.options.strokeColor,this.options.strokeColor=this.strokeGradient(o,i,s,n),this},e}(),window.AnimationUpdater={elements:[],animId:null,addAll:function(t){var i,e,s,n;for(n=[],e=0,s=t.length;e<s;e++)i=t[e],n.push(AnimationUpdater.elements.push(i));return n},add:function(t){return AnimationUpdater.elements.push(t)},run:function(t){var i,e,s,n;if(null==t&&(t=!1),isFinite(parseFloat(t))||!0===t){for(i=!0,e=0,s=(n=AnimationUpdater.elements).length;e<s;e++)n[e].update(!0===t)&&(i=!1);return AnimationUpdater.animId=i?null:requestAnimationFrame(AnimationUpdater.run)}if(!1===t)return!0===AnimationUpdater.animId&&cancelAnimationFrame(AnimationUpdater.animId),AnimationUpdater.animId=requestAnimationFrame(AnimationUpdater.run)}},\"function\"==typeof window.define&&null!=window.define.amd?define(function(){return{Gauge:n,Donut:s,BaseDonut:i,TextRenderer:a}}):\"undefined\"!=typeof module&&null!=module.exports?module.exports={Gauge:n,Donut:s,BaseDonut:i,TextRenderer:a}:(window.Gauge=n,window.Donut=s,window.BaseDonut=i,window.TextRenderer=a)}).call(this);";
static const char mainJs[] PROGMEM = "var speed,squal,stopupdate=1,spinnerarray=new Object,params=[],flag=!1;function getelement(e){return document.getElementById(e)}function showMessage(e){var t=getelement(\"info\");t.innerHTML=e,t.style.display=\"block\",setTimeout(function(){t.style.display=\"none\"},3e3)}function buttonreq(e){stopupdate=0;var t=new XMLHttpRequest,a=\"\",a=\"Stop\"==e.value?(t.open(\"GET\",\"buttonreq?cmd=0\"),\"Start\"):(t.open(\"GET\",\"buttonreq?cmd=1\"),\"Stop\");t.onload=function(){200===t.status&&(e.value=a),stopupdate=1},t.send()}function enableTextBox(e,t){0==t?(getelement(e).disabled=!0,getelement(e+\"label\").style.color=\"#aaa\",spinnerarray[e].disabled=!0):1==t&&(getelement(e).disabled=!1,getelement(e+\"label\").style.color=\"#000\",spinnerarray[e].disabled=!1)}function SelectChanged(e){if(\"runout\"==e.name)switch(parseInt(e.value)){case 0:enableTextBox(\"sqvalid\",0);break;case 1:enableTextBox(\"sqvalid\",1);break;default:enableTextBox(\"sqvalid\",0)}else if(\"jam\"==e.name)switch(parseInt(e.value)){case 0:enableTextBox(\"speedthid\",0),enableTextBox(\"timethid\",0);break;case 1:case 2:case 3:case 4:enableTextBox(\"speedthid\",1),enableTextBox(\"timethid\",1);break;default:enableTextBox(\"speedthid\",0),enableTextBox(\"timethid\",0)}}function getParams(){var a=new XMLHttpRequest;a.open(\"GET\",\"getparams\"),a.onload=function(){var e,t;200===a.status&&(e=JSON.parse(a.responseText),(t=[])[0]=parseInt(e.jam),t[1]=parseInt(e.runout),t[2]=parseInt(e.speedth),t[3]=parseInt(e.jamtimeout),t[4]=parseInt(e.minsq),t[5]=parseInt(e.led),params[4]-parseInt(t[4])!=0&&(getelement(\"sqvalid\").value=e.minsq,params[4]=parseInt(t[4])),params[2]-parseInt(t[2])!=0&&(getelement(\"speedthid\").value=e.speedth,params[2]=parseInt(t[2])),params[3]-parseInt(t[3])!=0&&(getelement(\"timethid\").value=e.jamtimeout,params[3]=parseInt(t[3])),params[5]-parseInt(t[5])!=0&&(getelement(\"brightnessid\").value=e.led,params[5]=parseInt(t[5])),params[1]-parseInt(t[1])!=0&&(params[1]=parseInt(t[1]),getelement(\"runoutsel\").value=e.runout,SelectChanged(getelement(\"runoutsel\"))),params[0]-parseInt(t[0])!=0&&(params[0]=parseInt(t[0]),getelement(\"jamsel\").value=e.jam,SelectChanged(getelement(\"jamsel\"))))},a.send()}function getData(){var a=new XMLHttpRequest;a.open(\"GET\",\"getdata\"),a.onload=function(){var e,t;200===a.status&&(e=JSON.parse(a.responseText),t=\"Running\",speed.set(Number(e.speed)),getelement(\"speedval\").innerHTML=e.speed,squal.set(Number(e.current)),getelement(\"sqval\").innerHTML=e.current,0==e.running?(getelement(\"status\").style.color=\"#739900\",t=\"Running\"):1==e.running?(getelement(\"status\").style.color=\"#6600cc\",t=\"Jammed\"):2==e.running?(getelement(\"status\").style.color=\"#cc0000\",t=\"Run out\"):3==e.running&&(getelement(\"status\").style.color=\"#e6e600\",t=\"Idle\"),getelement(\"status\").innerHTML=t)},a.send()}function setData(){var e=new XMLHttpRequest;params[0]=getelement(\"jamsel\").value.toString(),params[1]=getelement(\"runoutsel\").value.toString();var t=Math.floor(1e5*Math.random()).toString();(\"\"==getelement(\"jamsel\").value||parseInt(getelement(\"jamsel\").value)<0||4<parseInt(getelement(\"jamsel\").value))&&(params[0]=\"0\"),(\"\"==getelement(\"runoutsel\").value||parseInt(getelement(\"runoutsel\").value)<0||1<parseInt(getelement(\"runoutsel\").value))&&(params[1]=\"1\"),params[2]=parseInt(getelement(\"speedthid\").value).toString(),params[3]=parseInt(getelement(\"timethid\").value).toString(),params[4]=parseInt(getelement(\"sqvalid\").value).toString(),params[5]=parseInt(getelement(\"brightnessid\").value).toString(),e.open(\"GET\",\"setdata?jam=\"+params[0]+\"&runout=\"+params[1]+\"&speedth=\"+params[2]+\"&timeth=\"+params[3]+\"&minsqval=\"+params[4]+\"&brightness=\"+params[5]+\"&\"+t),e.onload=function(){200===e.status?\"saved\"==e.responseText&&showMessage(\"Saved successfully.\"):alert(e.status)},e.send()}function createSpinners(){var e=[\"speedthid\",\"timethid\",\"sqvalid\",\"brightnessid\"];for(i=0;i<4;i++){var t=new ISpin(getelement(e[i]),{min:0,wrapperClass:\"ispin-wrapper\",buttonsClass:\"ispin-button\",step:1,pageStep:10,disabled:!1,repeatInterval:200,wrapOverflow:!1,parse:Number,format:String,disabled:i<3});spinnerarray[e[i]]=t}}function createGauge(){var e={lines:12,angle:.15,lineWidth:.44,pointer:{length:.7,strokeWidth:.035,color:\"#000000\"},limitMax:\"false\",percentColors:[[0,\"#a9d70b\"],[.5,\"#f9c802\"],[1,\"#ff0000\"]],strokeColor:\"#E0E0E0\",generateGradient:!0},t=getelement(\"chart\");(speed=new Gauge(t).setOptions(e)).maxValue=500,speed.animationSpeed=32,speed.set(200);t=getelement(\"chart1\");(squal=new Gauge(t).setOptions(e)).maxValue=1023,squal.animationSpeed=32,squal.set(42)}setInterval(function(){1==stopupdate&&getData(),getParams()},1e3);";
static const char staticHTML1[] PROGMEM = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><link rel=\"stylesheet\" href=\"ispin.css\"><script type='text/javascript' src='gauge.min.js'></script><script type='text/javascript' src='ispin.min.js'></script><script type='text/javascript' src='main.min.js'></script></head><body style='font-family:sans-serif;' onload='createGauge();createSpinners();getParams()'><div style='margin:auto;width:925px;padding: 10px;margin-top: 115px;border-radius: 5px;-moz-border-radius: 5px;-webkit-border-radius: 5px;border: 2px solid #808080;'><div style='display: block;font-size: 180%;font-weight: bold;margin-bottom: 10px;'> Optifil Sensor Status : <span id='status'>Idle</span></div><div style='display: block;float:right;width:50;line-height: 1.6;'><form action='setdata' method='get'>Runout Detection <select name=\"runout\" id=\"runoutsel\" style='margin-left:10px;width:109px' onChange='SelectChanged(this)'>";
static const char staticHTML2[] PROGMEM = "</div><div style='display:block;line-height:1.6'><div id='info' style='display:block;width:150px;line-height:1.6;float:left'></div><input type='button' value='Save' onClick='setData()' style='width:80px;float:right;height:25px'></div></form><br></div><div><div style='overflow : hidden;width:318px;height:215px;float:left;'><canvas id='chart' style='display: block;width:370px'></canvas> <span style='font-size : 125 % ;font-weight: bold;margin-left: 95px' ;>Filament Speed&nbsp;:&nbsp;<span id='speedval'>10</span></span></div><div style='overflow : hidden;width:318x;height:215px'><canvas id='chart1' style='display : block;width:370px'></canvas> <span style='font-size : 125 % ;font-weight: bold;margin-left: 97px' ;>Surface Quality&nbsp;:&nbsp;<span id='sqval'>10</span></span></div></div></div></body></html>";
static const unsigned char firmware_data[3070] PROGMEM =
{   
  0x03,0xa4,0x6e,0x16,0x6d,0x41,0xf4,0x8f,0x95,0x6a,0xa1,0xe1,0x5c,0xeb,0xb1,0x2a,0xa0,0xb4,0x16,0x82,0x62,0x71,0x5d,0x0b,0xbe,0x01,0xeb,0x4a,0x61,0x54,0x42,0x98,0x76,0x63,0x98,0xb8,0x60,0x4a,0xe1,0x17,0x27,0x99,0x3f,0xce,0x9d,0x88,0x9b,0x4b,0x6d,0x31,0xc7,0xe7,0xba,0xa2,0x66,0x88,0xb5,0xb4,0xd2,0xb5,0x6b,0xd8,0xcd,0x91,0x5b,0xea,0xd2,0x9c,0x29,0x72,0xcb,0xeb,0x1e,0x60,0xe7,0x26,0x3f,0xf5,0xfc,0x6c,0xd3,0x11,0x0c,0xa9,0x51,0x30,0x4c,0xd3,0x77,0x37,0x2b,0x6f,0x13,0x89,0x7f,0x44,0x22,0xa7,0xcc,0xfa,0x55,0x09,0x90,0xa2,0xc6,0x0e,0x9e,0x9f,0xbc,0xfa,0x59,0x2f,0xd6,0x36,0xc2,0xfd,0x78,0x72,0x47,0xed,0x39,0xd1,0x20,0xc2,0xe7,0x4c,0xfb,0x74,0x6c,0x51,0x26,0xb6,0xf9,0x7a,0x57,0x0d,0x98,0xb2,0xe6,0x4e,0x1e,0x9f,0x9d,0xbc,0xd3,0x24,0xca,0xf7,0x6c,0x5a,0x1d,0xac,0xb0,0xf4,0x4b,0x14,0x8b,0x94,0xaa,0xb7,0xcd,0xf9,0x61,0x62,0x46,0x0e,0x9e,0x9f,0x92,0xa8,0xe0,0x54,0xf9,0x39,0xd1,0x01,0x63,0xa5,0x42,0x74,0xeb,0x81,0x6a,0xa6,0x9a,0x37,0x1f,0x8a,0x81,0x70,0xf9,0xd0,0x3c,0xd3,0xce,0xea,0xf5,0xd2,0x5c,0x7d,0xc2,0x76,0x3d,0x40,0x5e,0xd9,0xc6,0x03,0x11,0x19,0x66,0xf2,0xa7,0xd2,0xfa,0xa2,0xaf,0xf4,0x6c,0x02,0x6a,0x5a,0x06,0x00,0xe0,0x72,0x93,0x91,0x90,0x7b,0x19,0x80,0xbb,0x0b,0x9e,0xcc,0x9a,0x50,0xc9,0x80,0x91,0x29,0x8e,0xc1,0xe4,0xc4,0xfb,0xb7,0x9b,0xf5,0xf8,0x57,0xde,0xe9,0x39,0x71,0xbc,0x61,0x55,0x34,0x08,0xe1,0xb2,0x76,0xac,0xb7,0x50,0xa5,0xcc,0xa9,0x27,0x96,0x0d,0x31,0x89,0x8a,0x00,0x4c,0x46,0x17,0x1c,0x83,0xb2,0x47,0x08,0x8f,0xfe,0xe9,0x55,0xdf,0x07,0x49,0xb7,0xa4,0xd0,0x96,0x87,0xd0,0x1a,0x06,0xb7,0xda,0x97,0xa6,0x2e,0x9d,0x2f,0x6f,0xc7,0xb0,0x8d,0x7a,0x05,0x99,0x07,0x28,0x80,0x2b,0x37,0xce,0x4b,0x47,0x5a,0x4f,0x21,0x16,0xf4,0xea,0x44,0x5c,0x8f,0x37,0x19,0x9e,0x4a,0x0a,0x7e,0x7c,0xda,0x09,0xd8,0xb9,0x04,0x8f,0x68,0xcc,0x6e,0x8d,0x6d,0x46,0xb4,0x71,0xe4,0x13,0x50,0xd2,0x27,0xa5,0x14,0x86,0xc6,0x8c,0xf8,0x19,0xc4,0xc6,0xb3,0xae,0xbc,0x7a,0x93,0x9a,0xf4,0x59,0x9e,0xf6,0x3f,0x7f,0x92,0x21,0x9d,0xec,0xa7,0xe9,0x63,0xa6,0xb8,0x39,0xfb,0xd6,0x8e,0x4c,0x0d,0xa8,0x1e,0xf6,0x63,0xaf,0x5d,0x6e,0x12,0x13,0xcc,0x84,0xcb,0xcb,0x17,0x99,0x99,0xa9,0x33,0x35,0xf8,0x99,0xd4,0xe2,0x47,0x9b,0x3b,0x25,0xed,0x9b,0xe1,0x70,0x05,0xb9,0xc0,0xbe,0x9b,0xee,0xfa,0xd5,0x85,0xe2,0x65,0x3c,0x77,0x44,0x4b,0x24,0x7f,0x68,0xd6,0xac,0xbd,0x22,0x20,0x46,0x82,0xec,0x98,0xe7,0xc1,0x49,0x51,0x50,0x26,0x6d,0xa0,0xde,0xa1,0xbb,0xb1,0x5a,0x6f,0x2b,0x2e,0xaa,0xd4,0x7a,0xb9,0xd4,0x7b,0xec,0xe7,0xb1,0x79,0x79,0xa1,0x1b,0xa2,0xa5,0xea,0xb0,0xa2,0x54,0xea,0x26,0x67,0xb4,0xff,0xbe,0x8a,0xbf,0x64,0x2a,0x5e,0x6f,0x87,0xd7,0x5d,0x69,0xe7,0x9c,0xeb,0xe7,0x40,0xe9,0x92,0xad,0xc9,0xa6,0xde,0x6f,0xed,0x46,0x1f,0xf5,0x44,0x10,0xf2,0x8e,0x73,0x2f,0xff,0xc8,0x9f,0xf5,0x69,0x41,0x54,0x3e,0x3a,0xf4,0xe8,0xf3,0x60,0xae,0xe0,0x3c,0x5c,0xdf,0x5e,0xd9,0xd3,0xd8,0xdb,0x56,0xcb,0xf6,0x2a,0xcc,0x8d,0x1f,0x11,0x0a,0xb5,0xbd,0x75,0x40,0x22,0xd7,0x59,0x26,0x2f,0x1c,0x1d,0xf8,0x78,0xa3,0xfa,0xc8,0xb4,0x0f,0xbc,0x19,0x51,0xf9,0x99,0xb1,0x01,0x60,0xd3,0x26,0x63,0x79,0x52,0xc9,0xdd,0x69,0x25,0xe5,0xca,0x60,0x63,0x17,0x0b,0xe0,0x08,0x12,0xa2,0x05,0xe6,0x0d,0x50,0x2d,0x9c,0x9b,0x7c,0x47,0xec,0x90,0x6f,0x6c,0x2f,0xf1,0xe2,0x11,0x81,0xd3,0x83,0xf0,0x28,0x73,0x4f,0x22,0xc9,0x53,0xac,0xf4,0x1e,0xef,0xcb,0x27,0xfd,0xee,0x6b,0x64,0xf4,0xd9,0x0d,0x04,0x2d,0xf3,0x2b,0x0f,0xc7,0x9c,0x8a,0x21,0xf1,0x50,0x70,0x6f,0x6d,0x63,0xd6,0xa3,0xa9,0xba,0xd5,0x7d,0xd4,0x82,0xc7,0x3c,0x8f,0x69,0xb5,0x0b,0xf3,0xbe,0x2f,0x76,0xe2,0x2c,0xf9,0x25,0x45,0x41,0x41,0x11,0xd5,0x3e,0x1f,0x3c,0x7c,0xdb,0x1c,0x4c,0x9e,0xdc,0xf9,0xe1,0xc0,0xeb,0xc1,0xf5,0x0a,0x76,0x1e,0xfe,0x71,0xbb,0x30,0x82,0x60,0x13,0xe4,0x44,0xf2,0xb2,0x67,0xa8,0x83,0x25,0xc0,0xf6,0x5b,0x63,0x5f,0x9a,0x8f,0xba,0xc0,0xcf,0x0d,0x8c,0x1d,0x4f,0xb8,0x4f,0x2b,0x50,0x69,0xd0,0x0d,0x81,0xbf,0xf7,0xbb,0xef,0xfa,0x48,0x3a,0x21,0x0d,0xa8,0xe7,0xaa,0x61,0xc7,0x94,0x7c,0xbc,0x54,0x8b,0x73,0x8c,0xc7,0x76,0xe9,0x8c,0x85,0x3f,0xf9,0x32,0x16,0x58,0x55,0x39,0x14,0x21,0x31,0x34,0xeb,0x10,0xbf,0x08,0xf1,0x96,0x77,0xc5,0x08,0x33,0xc9,0xec,0x1b,0x7f,0xb8,0x53,0x2c,0x2e,0x0a,0x81,0x90,0x6c,0xb6,0x3f,0x8a,0x76,0x99,0x1e,0xe7,0x56,0x7c,0xdd,0xee,0x15,0x89,0x8c,0x65,0x61,0x82,0xb0,0x22,0x0b,0xa4,0x9d,0xb9,0x26,0x60,0x3a,0x2c,0x4b,0xb5,0x3e,0xf5,0x28,0xaf,0x02,0xc4,0x52,0x87,0x82,0x6e,0x1d,0x2f,0x8d,0x39,0xd9,0xd4,0x08,0x24,0xba,0x36,0xcd,0x09,0xf2,0xf1,0x10,0x03,0x69,0xbc,0xd9,0xc5,0xa9,0xdc,0xe7,0xfc,0x75,0xc5,0xa9,0xd3,0xc6,0xca,0xee,0xce,0xf8,0xee,0xde,0x1d,0x58,0xd7,0xd4,0x3b,0x55,0x09,0x95,0x96,0x90,0x25,0x16,0x83,0x93,0x05,0x6a,0xb4,0x94,0x04,0x2b,0xb7,0x2d,0x86,0xf2,0xeb,0x3b,0x25,0xf0,0x2e,0xcf,0x25,0x23,0x65,0x44,0x83,0x4c,0x09,0x52,0xa7,0x75,0xc9,0x1e,0x77,0xce,0xc9,0x41,0xa1,0xa9,0x55,0x6b,0x03,0x46,0x78,0x53,0xe4,0xc7,0x0c,0x7b,0xdf,0x36,0x23,0x4e,0x51,0x16,0xb3,0xcf,0xd3,0x97,0xbc,0x9a,0x01,0xa7,0xbb,0x66,0x98,0x68,0x7c,0x13,0x65,0xad,0x50,0x63,0x71,0x7e,0x9e,0x7f,0xed,0xf8,0x75,0x50,0x8f,0x24,0xba,0x56,0x09,0x2e,0x8f,0x3c,0xf0,0x96,0x3a,0x00,0x0c,0x2e,0x0c,0x96,0x82,0x89,0xdb,0xa9,0xdd,0xe8,0x3c,0x08,0xaf,0x86,0xf8,0x0a,0xb4,0x34,0xe1,0xeb,0x6e,0x68,0x3c,0xa1,0xe1,0xa1,0x8a,0xcb,0x85,0xc7,0xac,0x8c,0xd6,0x30,0xcd,0xbc,0x3b,0xb6,0x28,0x5d,0xcc,0x47,0xf8,0x16,0xaf,0x1a,0x2e,0x26,0x8f,0x3c,0xe6,0xf5,0x76,0xc4,0x59,0xc7,0xc2,0xf5,0xf2,0x6e,0xae,0x2c,0xfb,0xa5,0xac,0x7a,0x6a,0xcc,0x8e,0x68,0x7d,0x08,0x32,0xdb,0xae,0x4a,0x45,0x8e,0xb6,0xee,0xfe,0x5a,0x9e,0x7f,0xff,0x79,0x90,0xa3,0xc8,0x16,0x2e,0x04,0xee,0x5e,0x65,0xeb,0x73,0xe6,0xbb,0x34,0xc6,0xf5,0x63,0x57,0x49,0x29,0x71,0x72,0xd7,0x9f,0x07,0xab,0x2b,0x04,0xeb,0x2f,0xb1,0x58,0xa6,0x9d,0x87,0x79,0x7d,0x21,0x01,0x18,0xec,0xf2,0x93,0xfa,0x43,0x7b,0x68,0x2d,0xe4,0x35,0xc1,0x54,0x74,0xec,0xb1,0xbe,0xe3,0x1a,0xfe,0x4b,0x6b,0x72,0x0f,0xe3,0x78,0x2a,0xba,0xb2,0xbc,0x7c,0x81,0xdb,0x74,0x9f,0x1c,0x89,0xe1,0x2b,0x60,0x01,0xb2,0xcd,0x8f,0xcd,0x99,0x81,0xd4,0x2a,0xf6,0xcc,0x49,0x1a,0xe1,0x30,0x42,0x0d,0x1b,0x6d,0x29,0x45,0xf4,0x25,0x3d,0xcb,0x21,0x79,0xd0,0x0f,0xf0,0x4a,0x66,0x2c,0x09,0x53,0xe1,0x30,0x23,0xca,0x9f,0x94,0xda,0x14,0x98,0x09,0x25,0xcc,0x27,0x11,0x69,0x8c,0xb2,0x4c,0xfe,0x6c,0x3a,0x7f,0xbd,0x04,0x03,0xf1,0x09,0x93,0xff,0xc5,0x80,0x6b,0xc5,0xf0,0x79,0xac,0xd3,0x8e,0x1b,0xc9,0xcd,0x70,0xa3,0x38,0x1a,0x62,0x0e,0x76,0x42,0x64,0x02,0xc9,0xd4,0x17,0x84,0x56,0xb6,0x62,0xfe,0xfa,0x01,0xa4,0xae,0xa1,0x8f,0xa7,0x08,0xab,0x54,0x0f,0x81,0xc3,0xd3,0x54,0x6a,0x5c,0xfb,0x37,0x7b,0x04,0x2b,0xb9,0x1c,0xab,0x75,0x4f,0x81,0x97,0xa2,0xa4,0xf8,0x83,0xa5,0xed,0xe1,0xc0,0x00,0x93,0xe7,0xfa,0xe7,0xcc,0xfb,0x25,0x23,0x53,0x67,0x7e,0x9f,0xbd,0xfc,0x22,0x67,0x44,0xee,0x3c,0x6d,0x28,0x52,0x09,0x78,0xd0,0x94,0xda,0x76,0x7d,0xcb,0xa0,0xdc,0x70,0xd1,0x60,0x33,0x51,0x54,0xe7,0x50,0x62,0x36,0xbc,0x70,0x56,0x3f,0x3c,0xec,0x06,0x43,0x1d,0xc8,0x92,0x90,0x1d,0x5c,0xf3,0x60,0xfb,0xb4,0xdf,0x01,0xf0,0xc3,0xf0,0x38,0x9f,0x67,0x48,0x4a,0xb7,0xd7,0x0d,0xc9,0xb1,0xfb,0xb6,0x9e,0xfe,0x66,0x8a,0xc7,0x8c,0x62,0xa1,0xb0,0x22,0xd8,0xfa,0xe7,0xcc,0xe4,0xa1,0xb0,0x43,0x3c,0x9f,0x9e,0x08,0xc3,0x84,0x55,0xc1,0x62,0x11,0xb1,0x41,0x28,0x97,0x8e,0x49,0x53,0x92,0x84,0x5d,0x1a,0x21,0xa2,0x70,0x21,0x97,0x8e,0x28,0x83,0x04,0x4b,0xc1,0x62,0xf0,0xc0,0xb4,0xa9,0x87,0xcf,0xcb,0x57,0xbb,0x56,0x98,0xf1,0xf7,0x1c,0x1b,0xb7,0x7d,0xda,0xa1,0xb0,0x43,0xf9,0xdd,0x7b,0xc2,0x76,0xcf,0xbf,0xad,0x9b,0x23,0x95,0x09,0x4d,0xb5,0xab,0x62,0x36,0x2e,0xee,0xca,0x55,0x9e,0xaf,0x5c,0x0c,0x23,0x87,0x3a,0x67,0xce,0xbd,0x80,0xd3,0x87,0xcf,0x87,0xfc,0xba,0x97,0xf9,0xd2,0x90,0xd2,0x85,0xc8,0x8f,0xcd,0xbb,0x95,0xc4,0x35,0xc5,0x3a,0xa1,0x4f,0x94,0x15,0x53,0xaf,0x38,0xdf,0x41,0x49,0x11,0x5c,0xb1,0x30,0x8b,0x76,0x56,0x0c,0x2c,0xf4,0xb0,0x48,0xf6,0x43,0x79,0xf9,0xb1,0x1c,0x53,0xd3,0x2c,0x38,0xca,0x55,0x9e,0xaf,0x5e,0x5f,0x50,0xe6,0xb6,0x6f,0xf3,0xfb,0xd5,0x32,0xd5,0xaa,0xfb,0x81,0xfa,0x26,0x6f,0x9f,0xcf,0xcf,0x4d,0x18,0x41,0x66,0x06,0x44,0x0a,0xde,0xf5,0x09,0xd8,0xc7,0x21,0x6a,0x20,0x35,0xb3,0x37,0x9d,0x2e,0x7e,0x60,0x2b,0x1f,0x69,0x45,0xfd,0x47,0xb6,0x7f,0xcd,0xea,0x95,0x5c,0x57,0xa2,0xad,0x0b,0xe2,0x3d,0xac,0x62,0xb6,0x1e,0xee,0x09,0x1c,0x59,0x24,0xe4,0xa4,0xcf,0x08,0x74,0x12,0x80,0x88,0x86,0x80,0x3b,0x26,0xb9,0x64,0xd0,0x87,0x00,0xfa,0x4f,0xfc,0xdb,0x07,0x4c,0xdf,0xd4,0x58,0xce,0xc3,0x0b,0x2e,0x78,0x8a,0x7d,0x40,0xb6,0xf8,0x65,0x35,0xe8,0x33,0xc5,0x08,0x73,0x45,0xe9,0xd4,0x11,0x62,0xe4,0x94,0xd9,0x0d,0x6a,0x85,0x74,0x44,0x21,0xa3,0xd0,0x08,0x52,0xc4,0xac,0x38,0x85,0xaa,0x13,0x9e,0xfe,0x4d,0x4c,0xf3,0xb7,0x6d,0x51,0xd7,0xfd,0x28,0x85,0x25,0x0a,0x63,0x4e,0xdf,0x80,0x75,0x1d,0x29,0x52,0x45,0x81,0x14,0x01,0x82,0x9f,0xc0,0x8d,0xa3,0x56,0xa3,0x48,0x59,0x15,0xf1,0xcc,0x72,0x46,0x38,0xfa,0x18,0x13,0xa6,0x8d,0x77,0xc2,0x57,0x6c,0x19,0x31,0x1e,0x32,0x73,0x5f,0x8c,0x30,0x6e,0x15,0x8d,0xe0,0xcf,0x74,0x4a,0x20,0xeb,0x3a,0x36,0xd2,0xf8,0x9d,0x16,0xbf,0x5d,0x05,0x75,0xb7,0x60,0x6e,0x01,0x49,0x72,0xa4,0x2f,0x7b,0xec,0xda,0x39,0x69,0x8f,0xe2,0xe0,0x7d,0x33,0x52,0x44,0x41,0xa1,0x88,0x24,0x5e,0xe9,0xa2,0x61,0x7f,0xa6,0x54,0xa8,0x26,0x3f,0x3c,0xd7,0xb3,0x51,0xf7,0xbf,0x7a,0x66,0x47,0xa9,0xb8,0x53,0x36,0x6c,0x2f,0x2a,0x05,0x4e,0x0e,0x76,0xea,0x3e,0x5f,0x22,0x5c,0xd7,0x86,0xfe,0xbe,0xe1,0xd8,0xef,0xf5,0x67,0x0b,0x63,0xe5,0x76,0xec,0x66,0xa6,0x41,0xfe,0x36,0x02,0xd0,0x7c,0xe6,0xce,0xd7,0xb9,0xef,0x03,0x43,0xbc,0x0d,0xb9,0xce,0xbd,0x25,0x01,0xee,0xa0,0xab,0x28,0x75,0x7c,0x9a,0x32,0x8e,0x1e,0xb3,0x58,0x2d,0x87,0x2a,0x6e,0x88,0x52,0x39,0x53,0xd8,0xda,0x98,0xad,0x90,0x7f,0xfa,0x60,0x45,0xa9,0x99,0x11,0x92,0x26,0x11,0xff,0xda,0xaf,0x2b,0x75,0x56,0x8d,0x45,0xc1,0x8f,0x62,0x2f,0x01,0x27,0xfb,0x92,0x26,0x87,0x2d,0x87,0x14,0xb5,0x56,0xa9,0x68,0x84,0xab,0x8b,0x56,0xf3,0x8c,0x15,0x56,0x87,0x51,0xa4,0xd8,0xd4,0xa7,0x6d,0x68,0xc6,0x4d,0xb1,0xef,0x0d,0x18,0xa3,0x40,0x5d,0x06,0x49,0xc9,0xe2,0xed,0x07,0x2f,0xe0,0x8b,0xfa,0xe9,0x39,0xff,0x12,0x07,0x59,0x5e,0xf2,0xbd,0xfe,0xdd,0x5b,0x5c,0x22,0x71,0x8b,0x11,0x42,0x02,0xab,0x0a,0x9a,0x17,0xab,0x7c,0x65,0x17,0x0a,0xc8,0x99,0x86,0xac,0xb0,0x22,0xe7,0x0b,0x00,0x55,0x9a,0x11,0xbe,0x05,0x12,0x24,0x1f,0x2c,0x7b,0x51,0x8e,0x6b,0xa2,0x34,0x4d,0x29,0x38,0x97,0xc4,0x8a,0xbd,0x6b,0x21,0x36,0xdc,0xbc,0xea,0x5f,0xb8,0x9a,0x17,0xb4,0x7c,0x97,0x06,0xfe,0xbe,0xe7,0xf9,0xad,0x50,0x2d,0x7e,0x68,0xf3,0x7b,0xd7,0xf1,0x88,0x1d,0xa7,0xa5,0xe4,0x1c,0xe4,0xb7,0x2c,0xfa,0xf3,0x7b,0x2b,0x93,0x3d,0x0f,0x5c,0x25,0x6b,0x89,0x59,0xbf,0x02,0xef,0xa0,0x84,0x9e,0x5e,0xdb,0x5c,0x7a,0x52,0xab,0xea,0x09,0x36,0x56,0xb8,0x32,0xf9,0xd3,0xd8,0x3b,0x7b,0xeb,0x1c,0x06,0x29,0xc6,0xe8,0xd2,0x2f,0x7d,0x73,0xf5,0x77,0x33,0x63,0x1d,0x4f,0x3d,0xc6,0xcc,0xc7,0xe4,0xa4,0x15,0x01,0x7c,0xfc,0x6c,0x5d,0xd9,0x58,0x72,0x7f,0xe8,0x8d,0xc7,0xaa,0x6e,0x88,0xb3,0xda,0x95,0x54,0xc2,0x89,0x6e,0x37,0x31,0x45,0x1a,0x50,0xaf,0x5c,0x0c,0x23,0x87,0x44,0x24,0x9b,0xf4,0x5c,0xd3,0x1a,0x88,0x34,0x73,0x96,0x45,0x36,0x4d,0xa4,0x03,0xea,0x89,0xf9,0x9e,0xd0,0x83,0x5f,0xc3,0x98,0x25,0xaa,0x81,0x4b,0x72,0x85,0x4c,0x42,0xa2,0x46,0xde,0xd9,0xa4,0x40,0xfb,0x1a,0x17,0x99,0x3e,0x30,0x63,0x09,0x10,0xb7,0x60,0xb6,0x67,0xdc,0x9b,0xd7,0x7d,0xda,0xbe,0xb3,0x94,0x0b,0xa3,0x6f,0xc8,0xb9,0x09,0xfe,0xbe,0xe4,0xd8,0xfc,0xda,0x5a,0x97,0xb7,0x9d,0x4c,0x53,0xb4,0x4b,0xc9,0xbd,0x5a,0xbe,0x92,0xd6,0x6c,0x78,0x18,0x57,0x51,0x16,0xa0,0xc2,0x3b,0x5a,0x9d,0x41,0x4f,0xbd,0x99,0xf1,0x79,0x33,0x6c,0x74,0x8b,0x10,0x29,0xef,0x32,0x47,0xad,0xdd,0xcb,0xad,0x50,0xe0,0xd8,0x99,0xd8,0x63,0x0c,0x47,0xa4,0x82,0x40,0xe0,0x84,0x69,0xd5,0x16,0x90,0x04,0xd4,0x58,0xcd,0xb2,0x02,0x8a,0xeb,0x3c,0x1a,0x4a,0xfc,0x8b,0xdc,0xfd,0x44,0x70,0xc4,0x56,0x17,0xc5,0x2a,0x13,0xd5,0x49,0xd9,0x12,0x72,0x24,0x3c,0x58,0x9b,0x9a,0xa7,0x4c,0x5b,0x73,0x81,0x2a,0x08,0xfc,0xba,0xb6,0xab,0x27,0x54,0xa2,0x0c,0x90,0x09,0xd9,0x61,0x08,0x4f,0x95,0xa0,0x04,0x68,0x94,0x89,0x95,0x96,0xf1,0xc6,0x50,0x4c,0x98,0xd0,0x69,0xb5,0x42,0xe2,0x4a,0x8a,0xfe,0x9e,0x63,0xad,0x2f,0xb5,0x2f,0x01,0xdb,0x92,0x9b,0xec,0x52,0x44,0xcf,0x0c,0xbb,0x1c,0x98,0x47,0x2f,0x9a,0x21,0xdb,0x74,0x7f,0x11,0x9f,0xe1,0x07,0xdc,0x7a,0x15,0xc6,0x81,0xc3,0xcc,0x55,0x39,0x51,0x1a,0x47,0x52,0x7b,0xf3,0x34,0x2a,0xea,0xc4,0x64,0x28,0x7b,0x7b,0xe5,0xc8,0x30,0x63,0x9b,0x8a,0x51,0x1f,0xd2,0x64,0x09,0x1b,0x8d,0x6e,0xf4,0x8e,0x92,0xfa,0x1e,0x7f,0xa1,0x2a,0x46,0x46,0xe6,0x83,0xe6,0x06,0xc1,0x70,0xa2,0x85,0x49,0x4d,0x62,0xe0,0x7f,0x16,0x0f,0xdd,0x7f,0xb8,0x79,0x4f,0x53,0x85,0xc8,0x76,0xbc,0x83,0x2d,0x1a,0xec,0xf1,0xe9,0x41,0x69,0x6c,0xf2,0x0f,0x76,0x1a,0x7d,0x8a,0x93,0x54,0x6c,0xe3,0xe5,0x0b,0x45,0x4b,0xc3,0x55,0x68,0x4e,0xc2,0x5b,0x6f,0x1b,0x89,0x83,0x73,0xef,0xb8,0xfe,0x03,0xec,0xdb,0xc8,0xfb,0x83,0xcc,0xf0,0x4a,0x74,0xc1,0x4b,0xa1,0x0a,0xfd,0xf2,0xef,0x4d,0x50,0x1e,0x56,0x26,0x24,0x9f,0x77,0x7f,0x58,0xc2,0x40,0x3b,0x55,0x15,0x85,0xcb,0xa2,0xb6,0x2e,0x81,0x7a,0xd0,0x3c,0x50,0xf5,0x6b,0x41,0xe2,0x22,0x89,0x31,0xde,0xc3,0x3c,0xb9,0x78,0x7d,0xe9,0xd0,0x1a,0x00,0x45,0x56,0xe5,0x9f,0xbf,0x08,0x11,0xe5,0x07,0x0c,0x43,0x94,0x92,0x84,0x42,0x28,0x66,0x75,0xdb,0x86,0x2f,0xfe,0x1c,0x8f,0x85,0x9d,0x19,0xf2,0x27,0x18,0x8b,0x66,0x54,0x9a,0xe3,0xe5,0x14,0x34,0xfe,0xa7,0xf8,0xb2,0xda,0xaa,0x02,0xbf,0xca,0x97,0x99,0x3a,0xe2,0x77,0x2b,0x55,0x0e,0x32,0xf8,0xec,0x7f,0x2b,0x09,0xf1,0x49,0x90,0x02,0xd4,0x6e,0x81,0xfe,0xf9,0x4f,0x4d,0xf9,0xc7,0xee,0xe1,0x37,0x08,0x8e,0x68,0xb4,0x27,0xdd,0x67,0x4c,0xe9,0x68,0x5d,0xdd,0x63,0x18,0x9f,0x99,0xa2,0x15,0x08,0x54,0xa5,0xf7,0x13,0x60,0x55,0xb5,0x49,0x18,0x13,0x04,0x39,0xb4,0xf5,0x37,0x6b,0x8a,0xe6,0x2f,0x6b,0x57,0x72,0x70,0x86,0xb3,0xf3,0x82,0x6a,0x66,0xd0,0x41,0xd2,0xff,0x5b,0xe0,0xef,0xc6,0x73,0x9a,0xe3,0x11,0xf3,0x55,0x46,0x41,0xa0,0x86,0x24,0xbf,0x2b,0x44,0x85,0x6b,0x14,0xea,0x16,0x39,0x9a,0x2b,0x49,0xb0,0xfe,0x05,0x8c,0x46,0xc7,0x2f,0x4e,0x96,0xb2,0x39,0xd5,0x3f,0xfa,0x05,0x68,0x41,0xea,0xcf,0x7b,0x26,0xbc,0x3b,0xe8,0xc8,0x96,0x0d,0x7c,0x01,0x42,0x51,0x00,0xd7,0x09,0xa3,0x43,0x44,0xab,0x43,0x8f,0x41,0xbc,0x3b,0xc9,0x8a,0xa9,0xcb,0xc8,0xdb,0xf6,0xfc,0xd3,0x19,0x68,0xb8,0xc0,0x8f,0x08,0x32,0xf1,0xec,0x84,0x80,0x1e,0xb9,0x29,0x12,0x0c,0x29,0xcc,0x9c,0xd0,0x10,0xa7,0xd6,0x8a,0xe9,0xf6,0xfa,0x8d,0x7e,0x95,0x2e,0x57,0xcf,0x8e,0x38,0x6b,0x35,0xf0,0x2f,0xa6,0x4f,0x2a,0x7b,0x4b,0x1e,0x62,0x81,0x78,0xb0,0x48,0x40,0x7f,0xdb,0x7f,0x4e,0x1b,0x8f,0xf8,0x0d,0x1f,0x28,0x08,0x95,0x62,0xc0,0xea,0x94,0xf8,0xd4,0x92,0xa6,0xb7,0x32,0xba,0x36,0xc2,0xf8,0x66,0x57,0x89,0x44,0x8a,0x9b,0x6e,0x39,0x98,0xc2,0x86,0x62,0xb8,0x02,0x06,0x4d,0xeb,0x7f,0xfd,0xd9,0x18,0xe1,0x3c,0x52,0x91,0x80,0xe9,0xe5,0x41,0xff,0x12,0xe6,0x43,0xfa,0x69,0xf8,0x3b,0xba,0x48,0xd1,0xa8,0x0f,0x3c,0x2e,0x5e,0x12,0x58,0x97,0x25,0x5e,0x15,0x29,0xf0,0x05,0x33,0xc1,0x31,0xc5,0xaf,0x87,0x6b,0x9f,0xee,0x39,0x0d,0x12,0x76,0x44,0xeb,0xf6,0x59,0x71,0x6e,0xe0,0xd6,0x78,0xbc,0xd9,0x91,0x26,0x17,0x28,0xea,0x02,0xb4,0x96,0xc5,0x35,0xd9,0x31,0xd2,0x57,0x48,0x4b,0x69,0xa3,0x65,0x5b,0xf3,0xbd,0x79,0x46,0xb0,0x43,0xf6,0xac,0x9b,0x44,0xed,0x81,0xb4,0xa0,0xb5,0xa0,0xb1,0x48,0x02,0x08,0x01,0xd0,0x11,0xaf,0xe6,0x5d,0xc8,0x55,0x88,0xba,0x43,0xe3,0x4d,0x3c,0x81,0xb6,0x89,0x1a,0x4a,0xfb,0x6a,0xdd,0x88,0x7d,0xa1,0x8b,0x77,0xdf,0x95,0x95,0x35,0x38,0x80,0x77,0xdc,0xb0,0xeb,0x2b,0xc8,0x87,0xe8,0x02,0x2d,0xd9,0xb3,0x0f,0xf1,0xa1,0xe7,0x03,0xb5,0x29,0xf6,0xf3,0x10,0x92,0xe2,0x30,0xd2,0x86,0x85,0x91,0xd8,0xb5,0xaf,0x5b,0x73,0x23,0x43,0x62,0xc1,0x66,0x28,0xb4,0xad,0x5f,0x78,0x53,0x05,0x69,0x50,0x22,0xc6,0x0e,0x9e,0x9f,0x9d,0xb8,0xd3,0x05,0x69,0x50,0x03,0x65,0x29,0xb1,0xe0,0x42,0xe7,0x4c,0xfb,0x55,0x09,0x90,0xa2,0xc6,0xef,0x3d,0xf8,0x72,0x66,0x2f,0xdc,0x3a,0xf6,0x4f,0xfd,0x59,0x30,0xe2,0x27,0xcc,0xfb,0x55,0x09,0x71,0x41,0xe1,0x21,0xa1,0xa1,0xc0,0xe3,0x25,0xa9,0xd0,0x22,0xc6,0xef,0x5c,0x1b,0xb4,0xea,0x56,0x2e,0xbf,0xfc,0x5b,0x34,0xcb,0x14,0x8b,0x94,0xaa,0xb7,0xec,0x5a,0x36,0xcf,0x1c,0xba,0xf6,0x9f,0xdc,0x35,0xf3
};
bool configMode=false;
bool serverUp=false;
ESP8266WebServer server(80);

// Create CLI Object
SimpleCLI cli;

// Commands
Command getdata;
Command getparam;
Command setparam;
Command start;
Command stop;
Command help;

// Callback function for setparam command
void setparamCallback(cmd* c) {
	Command cmd(c); // Create wrapper object

	// Get arguments
	Argument SpeedThresholdArg = cmd.getArgument("speedth");
	Argument JamTimeoutArg = cmd.getArgument("timeout");
	Argument MinSQThresholdArg = cmd.getArgument("minsq");
	Argument LEDBrightnessArg = cmd.getArgument("led");
	Argument RunoutDetectionArg = cmd.getArgument("runout");
	Argument JamDetectionArg = cmd.getArgument("jam");
	// Get values
	int SpeedThresholdVal = SpeedThresholdArg.getValue().toInt();
	int JamTimeoutArgVal = JamTimeoutArg.getValue().toInt();
	int MinSQThresholdVal = MinSQThresholdArg.getValue().toInt();
	int LEDBrightnessVal = LEDBrightnessArg.getValue().toInt();
	int RunoutDetectionVal = RunoutDetectionArg.getValue().toInt();
	int JamDetectionVal = JamDetectionArg.getValue().toInt();

	zeroSpeedTh = SpeedThresholdVal==-1? zeroSpeedTh: zeroSpeedTh;
	zeroSpeedTime = JamTimeoutArgVal == -1 ? zeroSpeedTime: JamTimeoutArgVal;
	minSqVal = MinSQThresholdVal == -1 ? minSqVal: MinSQThresholdVal;
	LEDBrightness = LEDBrightnessVal == -1 ? LEDBrightness: LEDBrightnessVal;
	RunOutDetection = RunoutDetectionVal == -1 ? RunOutDetection: RunoutDetectionVal;
	JamDetection = JamDetectionVal == -1 ? JamDetection: JamDetectionVal;

	saveSettings();
	if (RunOutDetection == 0)
		startMonitoring = false;
	else
		startMonitoring = true;
}

void getdataCallback(cmd* c) 
{
	Command cmd(c); // Create wrapper object
	int runstatus = 0;
	if (jammed)
		runstatus = 1;
	if (runout)
	{
		runstatus = 2;
		rspeed = 0;
	}
	if (!startMonitoring)
		runstatus = 3;
	Serial.printf("{\"squal\":%d,\"dist\":%Ld,\"speed\":%Ld,\"running\":%d,\"current\":%d}", Squal, rdist, rspeed, runstatus, CurrentSensorValue);
}

void getparamCallback(cmd* c)
{
	readSettings();
	Serial.printf("{\"speedth\":%d,\"jamtimeout\":%Ld,\"minsq\":%Ld,\"led\":%d,\"runout\":%d,\"jam\":%d}", zeroSpeedTh, zeroSpeedTime, minSqVal, LEDBrightness, RunOutDetection, JamDetection);
}

void startCallback(cmd* c)
{
	Serial.println("start sensor");
}
void stopCallback(cmd* c)
{
	Serial.println("stop sensor");
}
void helpCallback(cmd* c)
{
	Serial.println("Sensor Help");
}

// Callback in case of an error
void errorCallback(cmd_error* e) {
	CommandError cmdError(e); // Create wrapper object

	Serial.print("ERROR: ");
	Serial.println(cmdError.toString());

	if (cmdError.hasCommand()) {
		Serial.print("Did you mean \"");
		Serial.print(cmdError.getCommand().toString());
		Serial.println("\"?");
	}
}
void setupCommand()
{
	setparam = cli.addCommand("setparam", setparamCallback);
	setparam.addArgument("speedth", "-1");
	setparam.addArgument("timeout", "-1");
	setparam.addArgument("minsq", "-1");
	setparam.addArgument("led", "-1");
	setparam.addArgument("runout","-1");
	setparam.addArgument("jam","-1");

	getdata = cli.addCommand("getdata", getdataCallback);
	getparam = cli.addCommand("getparam", getparamCallback);
	start = cli.addCommand("start", startCallback);
	stop = cli.addCommand("stop", stopCallback);
	help = cli.addCommand("help", helpCallback);

}

void configModeCallback (WiFiManager *myWiFiManager) 
{
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());

    Serial.println(myWiFiManager->getConfigPortalSSID());
    configMode=true;
	setLedColor(0, 1, 0.6);
}

void setup() {
    configMode=false;
    serverUp=false;
    Serial.begin(115200);
    delay(1000);
    pinMode (button0, INPUT);
    pinMode (button1, INPUT);
    pinMode (ncs, OUTPUT);
    pinMode (runoutLed, OUTPUT);
    pinMode (jamLed, OUTPUT);
    pinMode (notifyPin, OUTPUT);
    digitalWrite(runoutLed, HIGH);
    digitalWrite(jamLed, LOW);
    digitalWrite(notifyPin, HIGH);
    
    SPI.begin();
    SPI.beginTransaction(ADNS);
    performStartup();
    Serial.println("ADNS9800 Polling");
    dispRegisters();
    delay(1000);
    initComplete=9;
    SPI.endTransaction();

    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
	strip.setPixelColor(0, 16, 16, 0);
	strip.show();
	setLedColor(0, 0.75, 0.6);


    WiFiManager wifiManager;
    wifiManager.autoConnect("Optifil Sensor","1234567890");
    wifiManager.setAPCallback(configModeCallback);
	
    server.on("/", handleRoot);               // Call the 'handleRoot' function when a client requests URI "/"
    server.on("/getdata", handleGetData); 
	server.on("/getparams", handleGetParams);
    server.on("/setdata", handleSetData); 
    //server.on("/buttonreq", handleBtnReq);
    server.on("/gauge.min.js", handleGaugeJsData);
	server.on("/ispin.min.js", handleIspinJsData);
	server.on("/main.min.js", handleMainJsData);
	server.on("/ispin.css", handleIspinCssData);
    server.on("/description.xml", HTTP_GET, []() {
      SSDP.schema(server.client());
    });
    server.onNotFound(handleNotFound);        // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"    
    SPIFFS.begin(); 
    readSettings();
	setLedColor(0, 0, 0);
	setupCommand();
	if (RunOutDetection == 0)
		startMonitoring = false;
	else
		startMonitoring = true;
}

void adns_com_begin() {
    digitalWrite(ncs, LOW);
}

void adns_com_end() {
    digitalWrite(ncs, HIGH);
}

byte adns_read_reg(byte reg_addr) {
    adns_com_begin();

    // send adress of the register, with MSBit = 0 to indicate it's a read
    SPI.transfer(reg_addr & 0x7F );
    delayMicroseconds(100); // tSRAD
    // read data
    byte data = SPI.transfer(0xFF);

    delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
    adns_com_end();
    delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

    return data;
}

void adns_write_reg(byte reg_addr, byte data) {
    adns_com_begin();

    //send adress of the register, with MSBit = 1 to indicate it's a write
    SPI.transfer(reg_addr | 0x80 );
    //sent data
    SPI.transfer(data);

    delayMicroseconds(20); // tSCLK-NCS for write operation
    adns_com_end();
    delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound
}

void adns_upload_firmware() {
    // send the firmware to the chip, cf p.18 of the datasheet
    Serial.println("Uploading firmware...");
    // set the configuration_IV register in 3k firmware mode
    adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved

    // write 0x1d in SROM_enable reg for initializing
    adns_write_reg(REG_SROM_Enable, 0x1d);

    // wait for more than one frame period
    delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low

    // write 0x18 to SROM_enable to start SROM download
    adns_write_reg(REG_SROM_Enable, 0x18);

    // write the SROM file (=firmware data)
    adns_com_begin();
    SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
    delayMicroseconds(15);

    // send all bytes of the firmware
    unsigned char c;
    for(int i = 0; i < firmware_length; i++) {
        c = (unsigned char)pgm_read_byte(firmware_data + i);
        SPI.transfer(c);
        delayMicroseconds(15);
    }
    adns_com_end();
}


void performStartup(void) {
    adns_com_end(); // ensure that the serial port is reset
    adns_com_begin(); // ensure that the serial port is reset
    adns_com_end(); // ensure that the serial port is reset
    adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
    delay(50); // wait for it to reboot
    // read registers 0x02 to 0x06 (and discard the data)
    adns_read_reg(REG_Motion);
    adns_read_reg(REG_Delta_X_L);
    adns_read_reg(REG_Delta_X_H);
    adns_read_reg(REG_Delta_Y_L);
    adns_read_reg(REG_Delta_Y_H);
    // upload the firmware
    adns_upload_firmware();
    delay(10);
    //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
    // reading the actual value of the register is important because the real
    // default value is different from what is said in the datasheet, and if you
    // change the reserved bytes (like by writing 0x00...) it would not work.
    byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
    adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );

    delay(1);

    Serial.println("Optical Chip Initialized");
    CustomSetting();
}

void UpdatePointer(void) {
    if(initComplete==9) {

        digitalWrite(ncs,LOW);
        xydat[0] = (unsigned char)adns_read_reg(REG_Delta_X_L);
        xydat[1] = (unsigned char)adns_read_reg(REG_Delta_Y_L);
        digitalWrite(ncs,HIGH);
    }
}
void UpdateSqual(void) {
    if(initComplete==9) {
        digitalWrite(ncs,LOW);
        Squal = (unsigned char)adns_read_reg(REG_SQUAL);
        if(Squal<5)
          Squal=0;
        digitalWrite(ncs,HIGH);
    }
}

void CustomSetting(void) {
    if(initComplete==9)
    {
        adns_write_reg(REG_Snap_Angle,0x86);
        adns_write_reg(REG_Lift_Detection_Thr,0x08);
        adns_write_reg(REG_Configuration_I,0x24);   //7200 count per inch
        adns_write_reg(REG_Configuration_II,0x08);   //Fixed frame rate
        adns_write_reg(REG_Frame_Period_Upper,0x5d);  //run at max frame rate
    }
}
void dispRegisters(void) {
    int oreg[7] = {
        0x00,0x3F,0x2A,0x02
    };
    char* oregname[] = {
        "Product_ID","Inverse_Product_ID","SROM_Version","Motion"
    };
    byte regres;

    digitalWrite(ncs,LOW);

    int rctr=0;
    for(rctr=0; rctr<4; rctr++) {
        SPI.transfer(oreg[rctr]);
        delay(1);
        Serial.println("---");
        Serial.println(oregname[rctr]);
        Serial.println(oreg[rctr],HEX);
        regres = SPI.transfer(0);
        Serial.println(regres,BIN);
        Serial.println(regres,HEX);
        delay(1);
    }
    digitalWrite(ncs,HIGH);
}

int convTwosComp(int b) {
    //Convert from 2's complement
    if(b & 0x80) {
        b = -1 * ((b ^ 0xff) + 1);
    }
    return b;
}

void setLedColor(float R, float G, float B)
{
  if(R!=globalR || G!=globalG || B!=globalB)
  {
	  for(int i=0;i< numLEDS;i++)
		strip.setPixelColor(i, R*LEDBrightness, G*LEDBrightness, B*LEDBrightness);
      strip.show();    
  }
}

void handleRoot() 
{
  readSettings();
  String tempstring = "";
  String tempstring1 = "";

  if (RunOutDetection == 1)				//select the  appropriate option
	  tempstring = "<option value=\"0\">Disabled</option><option value=\"1\" selected>Surface Quality</option>";
  else if (RunOutDetection == 0)
	  tempstring = "<option value=\"0\" selected>Disabled</option><option value=\"1\">Surface Quality</option>";

  if (JamDetection == 0)
	  tempstring1 = "<option value=\"0\" selected>Disabled</option><option value=\"1\">Speed timeout</option><option value=\"2\">USB Input</option><option value=\"3\">Stepper Current</option><option value=\"4\">External input</option>";
  else if (JamDetection == 1)
	  tempstring1 = "<option value=\"0\">Disabled</option><option value=\"1\" selected>Speed timeout</option><option value=\"2\">USB Input</option><option value=\"3\">Stepper Current</option><option value=\"4\">External input</option>";
  else if (JamDetection == 2)
	  tempstring1 = "<option value=\"0\">Disabled</option><option value=\"1\">Speed timeout</option><option value=\"2\" selected>USB Input</option><option value=\"3\">Stepper Current</option><option value=\"4\">External input</option>";
  else if (JamDetection == 3)
	  tempstring1 = "<option value=\"0\">Disabled</option><option value=\"1\">Speed timeout</option><option value=\"2\">USB Input</option><option value=\"3\" selected>Stepper Current</option><option value=\"4\">External input</option>";
  else if (JamDetection == 4)
	  tempstring1 = "<option value=\"0\">Disabled</option><option value=\"1\">Speed timeout</option><option value=\"2\">USB Input</option><option value=\"3\">Stepper Current</option><option value=\"4\" selected>External input</option>";

  server.send(200, "text/html", String(staticHTML1) + tempstring + "</select><br><span id='sqvalidlabel' style='color:#aaa'>SQual Threshold</span><input id='sqvalid' type='text' name='sqval' size=10 value='" + String(minSqVal) + \
	  "' style='margin-left:19px;width:102px'><br>Jam Detection <select name=\"jam\" id=\"jamsel\" style='margin-left:32px;width:109px' onChange='SelectChanged(this)'>" + tempstring1 + \
	  "</select><br><span id='speedthidlabel' style='color:#aaa'>Speed Threshold</span><input id='speedthid' type='text' name='speedth' size=10 value='" + String(zeroSpeedTh) + "' style='margin-left:17px;width:102px' disabled><br><span id='timethidlabel' style='color:#aaa'>Time Threshold</span><input id='timethid' type='text' name='timeth' size=10 style='margin-left:28px;width:102px' value='" + String(zeroSpeedTime) + "' disabled><br>LED Brightness<input id='brightnessid' type='text' name='brightness' size=10 value='" + String(LEDBrightness) + "' style='margin-left:28px;width:102px'><div style='height: 15px;'>" + String(staticHTML2));
	  
	  
	  
	 // "'><br> Time Threshold&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<input id='timethid' type='text' name='timeth' size=2 style='margin-left:6px' value='"+ String(zeroSpeedTime) +"'><br> Min SQ Threshold&nbsp;&nbsp;<input id='sqvalid' type='text' name='sqval' size=2 value='" + String(minSqVal) + "'><br> LED Brightness&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<input id='brightnessid' type='text' name='brightness' size=2 value='" + String(LEDBrightness) + "'><br> <input type=\"checkbox\" id=\"extin\" value=\"1\" " + tempstring + "> Enable External Input<br> <input type=\"checkbox\" id=\"jamdetect\" value=\"1\" " + tempstring1 + "> Enable Jam Detection<div style='height: 15px;'></div><input type=\"button\" value=\"Save\" onClick='setData()' style='width:80px;float:right;'></form><br> <input type=\"button\" id=\"reqbutton\" value=\"Start\" onClick='buttonreq(this)' style='width : 80px;float: right;margin-top:3px'><br></div><div><div style='overflow : hidden;width:318px;height:215px;float:left;'><canvas id='chart' style='display: block;width:370px'></canvas> <span style='font-size : 125 % ;font-weight: bold;margin-left: 95px';>Filament Speed&nbsp;:&nbsp;<span id='speedval'>10</span></span></div><div style='overflow : hidden;width:318x;height:215px'><canvas id='chart1' style='display : block;width:370px'></canvas> <span style='font-size : 125 % ;font-weight: bold;margin-left: 97px';>Surface Quality&nbsp;:&nbsp;<span id='sqval'>10</span></span></div></div></div></body></html>");
}

void handleGetData() 
{
  static char tempstr[64]= {'\0'};
  //(((xDist*25.4)/3600))
  int runstatus=0;
  if(jammed)
    runstatus = 1;
  if (runout)
  {
	  runstatus = 2;
	  rspeed = 0;
  }
  if (!startMonitoring)
	  runstatus = 3;
  sprintf(tempstr, "{\"squal\":%d,\"dist\":%Ld,\"speed\":%Ld,\"running\":%d,\"current\":%d}", Squal,rdist,rspeed,runstatus, CurrentSensorValue);
  server.send(200, "application/json",tempstr );   // Send HTTP status 200 (Ok) and send some text to the browser/client
}
void handleGetParams()
{
	static char tempstr[80] = { '\0' };
	sprintf(tempstr, "{\"speedth\":%d,\"jamtimeout\":%Ld,\"minsq\":%Ld,\"led\":%d,\"runout\":%d,\"jam\":%d}", zeroSpeedTh, zeroSpeedTime, minSqVal, LEDBrightness, RunOutDetection, JamDetection);
	server.send(200, "application/json", tempstr);   // Send HTTP status 200 (Ok) and send some text to the browser/client
}

void saveSettings()
{
    //SPIFFS.remove("sensorSettings.txt");
    File f = SPIFFS.open("sensorSettings.txt", "w");
    f.println(String(zeroSpeedTh));
    f.println(String(zeroSpeedTime));
	f.println(String(minSqVal));
	f.println(String(LEDBrightness));
	f.println(String(JamDetection));
	f.println(String(RunOutDetection));
    f.close(); 
}
void readSettings()
{
    File f = SPIFFS.open("sensorSettings.txt", "r");
    if (f) 
    {
      zeroSpeedTh=f.readStringUntil('\n').toInt();
      zeroSpeedTime=f.readStringUntil('\n').toInt();
	  minSqVal= f.readStringUntil('\n').toInt();
	  LEDBrightness = f.readStringUntil('\n').toInt();
	  JamDetection = f.readStringUntil('\n').toInt();
	  RunOutDetection = f.readStringUntil('\n').toInt();
      f.close(); 
    }
    else
    {
      zeroSpeedTh=5;
      zeroSpeedTime=10;      
    }
}
void handleSetData() 
{
  if( !server.hasArg("speedth") || !server.hasArg("timeth") || server.arg("speedth") == NULL || server.arg("timeth") == NULL || !server.hasArg("jam") || server.arg("jam") == NULL || !server.hasArg("runout") || server.arg("runout") == NULL || !server.hasArg("minsqval") || server.arg("minsqval") == NULL || !server.hasArg("brightness") || server.arg("brightness") == NULL)
  { 
    server.send(400, "text/plain", "400: Invalid Request");         // The request is invalid, so send HTTP status 400
    return;
  }
  else
  {
  //save server.arg("speedth");
  //save server.arg("timeth")
   zeroSpeedTh=server.arg("speedth").toInt();
   zeroSpeedTime=server.arg("timeth").toInt();  
   minSqVal = server.arg("minsqval").toInt();
   LEDBrightness = server.arg("brightness").toInt();
   RunOutDetection = server.arg("runout").toInt();
   JamDetection = server.arg("jam").toInt();
   saveSettings();
   server.send(200, "text/html","saved");
   if (RunOutDetection == 0)
	   startMonitoring = false;
   else
	   startMonitoring = true;
  }
}

//void handleBtnReq() 
//{
//  if( !server.hasArg("cmd") || server.arg("cmd") == NULL) 
//  { 
//    server.send(400, "text/plain", "400: Invalid Request");         // The request is invalid, so send HTTP status 400
//    return;
//  }
//  else
//  {
//   int ret = server.arg("cmd").toInt();
//   if (ret == 0)
//	   startMonitoring = false;
//   else
//	   startMonitoring = true;
//   server.send(200, "text/html","saved");
//  }
//}

void handleNotFound(){
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

void handleGaugeJsData()
{
  server.send(200, "text/javascript", gaugeJs);
}
void handleMainJsData()
{
	server.send(200, "text/javascript", mainJs);
}
void handleIspinJsData()
{
	server.send(200, "text/javascript", ispinJs);
}
void handleIspinCssData()
{
	server.send(200, "text/css", ispinCss);
}
void CalcStats(uint8_t *variance, uint8_t *difference)
{
	uint8_t currentMin = 255;
	uint8_t currentMax = 0;
	uint32 sum1 = 0;
	uint32 sum2 = 0;
	for (int q = 0;q < QUEUE_SIZE_ITEMS;q++)
	{
		if (currentQueue[q] < currentMin)
			currentMin = currentQueue[q];
		if (currentQueue[q] > currentMax)
			currentMax = currentQueue[q];
		sum1 += currentQueue[q];
		sum2 += currentQueue[q] * currentQueue[q];
	}
	//float avg = (float)sum1 / QUEUE_SIZE_ITEMS;
	//sum1 = round(avg);
	//Serial.println(sum1);
	//for (int q = 0;q < QUEUE_SIZE_ITEMS;q++)
	//{
	//	long diff = currentQueue[q]-sum1;
	//	diff *= diff;
	//	sum2 += (uint32)diff;
	//}
	sum1 *= sum1;
	float temp = (float)sum1 / (QUEUE_SIZE_ITEMS);
	//sum1 /= (QUEUE_SIZE_ITEMS-1);
	*variance = (sum2 -temp)/ (QUEUE_SIZE_ITEMS);
	*difference = currentMax - currentMin;
}
void loop() 
{
    currTime = millis();
    server.handleClient();    
	// Check if user typed something into the serial monitor
	if (Serial.available()) {
		// Read out string from the serial monitor
		String input = Serial.readStringUntil('\n');

		// Echo the user input
		Serial.print("# ");
		Serial.println(input);

		// Parse the user input into the CLI
		cli.parse(input);
	}

	if (cli.errored()) {
		CommandError cmdError = cli.getError();

		Serial.print("ERROR: ");
		Serial.println(cmdError.toString());

		if (cmdError.hasCommand()) {
			Serial.print("Did you mean \"");
			Serial.print(cmdError.getCommand().toString());
			Serial.println("\"?");
		}
	}
    if(currTime > pollTimer)
    {
        if(currTime > debounceTime)
        {
          if(!digitalRead(button0))
          {
			  if (prevButton0State)
			  {
				  startMonitoring = !startMonitoring;
				  RunOutDetection = (startMonitoring ? 1 : 0);
				  Serial.println("Button0 pressed : " + String(startMonitoring));
			  }
			  prevButton0State = false;
			  longpressCounter++;
			  if (longpressCounter >= 50)
			  {
				  Serial.println("Clear WiFi passwords");
				  WiFi.disconnect(true);
				  setLedColor(0, 0, 1);
				  delay(2000);
				  ESP.reset();
				  longpressCounter = 0;
			  }
          }
		  else
		  {
			  prevButton0State = true;
			  longpressCounter = 0;
		  }
		  //if (!digitalRead(button1))
		  //{
			 // Serial.println("Button1 pressed : ");
		  //}
          debounceTime=currTime+100;
        }
        if(!configMode)
        {
          if(!serverUp)
          {
            server.begin();                           // Actually start the server
            IPAddress ip; 
            ip = WiFi.localIP();
            String ipString = String(ip[0]);
            for (byte octet = 1; octet < 4; ++octet) {
              ipString += '.' + String(ip[octet]);
            }            
            Serial.print("HTTP server started : ");
			Serial.println("http://" + String(ipString) + "/");
            serverUp=true;
            Serial.printf("Starting SSDP...\n");
            SSDP.setDeviceType("upnp:rootdevice");
            SSDP.setSchemaURL("description.xml");
            SSDP.setHTTPPort(80);
            SSDP.setName("Optifil Filament Sensor");
            SSDP.setSerialNumber(ESP.getChipId());
            SSDP.setURL("http://"+String(ipString)+"/");
            SSDP.setModelName("Optifil Sensor");
            SSDP.setModelNumber("929000226503");
            SSDP.setModelURL("http://www.optifil.com");
            SSDP.setManufacturer("LP Electronics");
            SSDP.setManufacturerURL("http://www.logicalside.com");
            SSDP.begin();         

			if (!MDNS.begin("optifil")) {             // Start the mDNS responder for optifil.local
				Serial.println("Error setting up MDNS responder!");
			}
			Serial.println("mDNS responder started");
          }
            
        }
        //digitalWrite(testpin, tempLigh);
        //digitalWrite(ledpin, !tempLigh);
        //tempLigh=!tempLigh;
		uint8_t tempsensnorval = (uint8_t)(analogRead(sensorPin)/4);
		memcpy(&currentQueue[1], &currentQueue[0], sizeof(uint8_t) * (QUEUE_SIZE_ITEMS - 1));
		currentQueue[0] = tempsensnorval;
		//if (currentQueue.isFull())
		//	currentQueue.dequeue();
		//currentQueue.enqueue(tempsensnorval);
		//if (tempsensnorval < curreftMin)
		//	curreftMin = tempsensnorval;
		//if (tempsensnorval > curreftMax)
		//	curreftMax = tempsensnorval;
		//curreftDFF = abs(curreftMax - curreftMin);
		//if (curreftDFF > 10)
		//	currentSensnorCount++;
		//else
		//{
		//	currentSensnorCount--;
		//	if (currentSensnorCount < 0)
		//		currentSensnorCount = 0;
		//}
		//for (int q = 0;q < QUEUE_SIZE_ITEMS;q++)
		//{
		//	Serial.printf("(%d, %d) ", q, currentQueue[q]);
		//}
		//uint8_t v = 0, d = 0;
		//CalcStats(&v, &d);
		//Serial.printf("(%d, %d)\n", v, d);
		//Serial.println(minMaxDifference());
		//Serial.println("\n_______________________________________");
		CurrentSensorValue = tempsensnorval;
        UpdatePointer();
        UpdateSqual();
        xydat[0] = convTwosComp(xydat[0]);
        xydat[1] = convTwosComp(xydat[1]);
        xDist=0;
        yDist=0;
        if(xydat[0] != 0 || xydat[1] != 0)
        {
            xDist=xydat[0];
            yDist=xydat[1];
        }
        if(Squal<= minSqVal && startMonitoring)	//start monitoring is enabled
        {
            setLedColor(1,0,0);
            runout=true;     
            digitalWrite(notifyPin, LOW);   
        }
        else
        {    
			if (startMonitoring)
			{
				runout = false;
				if (!jammed)
				{
					digitalWrite(notifyPin, HIGH);
					setLedColor(0, 1, 0);
				}
			}
			else		//if not monitoring then don't notify error and show a yellow LED
			{
				digitalWrite(notifyPin, HIGH);
				setLedColor(1, 1, 0);
			}
        }
        rdist = sqrt(  xDist*xDist +  yDist*yDist);    
        if(!runout)																			//do jam detection only if the filament has not run out i.e. runout is false
        {
          if(distCount<100)
          {
            temprdist=temprdist+rdist;
            distCount++;
          }
          else if(distCount>=100)
          {
            rspeed=temprdist;
            distCount=0;
            temprdist=0;          
            if(rspeed<zeroSpeedTh)
              rspeedBelowTh++;
            else
              rspeedBelowTh=0;
			uint8_t v = 0, d = 0;
			CalcStats(&v, &d);
            if(rspeedBelowTh>=zeroSpeedTime && JamDetection==1 && startMonitoring)			//if timeout based jam detection is enabled and start monitoring is enabled
            {
              rspeedBelowTh=zeroSpeedTime;
              setLedColor(1,0,2);
              jammed=true;
              digitalWrite(notifyPin, LOW);
            }
			else if (rspeedBelowTh >= zeroSpeedTime && JamDetection == 3 && d > MAXDIFFERENCE && v> MAXVARIANCE && startMonitoring)			//if current sensor based jam detection is enabled and start monitoring is enabled
			{
				rspeedBelowTh = zeroSpeedTime;
				setLedColor(1, 0, 2);
				jammed = true;
				digitalWrite(notifyPin, LOW);
			}
            else
            {
				if (startMonitoring)
				{
					setLedColor(0, 1, 0);
					jammed = false;
					digitalWrite(notifyPin, HIGH);
				}
				else		//if not monitoring then don't notify error and show a yellow LED
				{
					digitalWrite(notifyPin, HIGH);
					setLedColor(1, 1, 0);
				}
            }
          }
        }
        else
        {
          rspeedBelowTh=0;
        }
        pollTimer = currTime + 10;
    }

}

