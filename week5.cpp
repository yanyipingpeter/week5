#include <iostream>
#include <cstdlib>
#include <queue>
#include <thread>
#include <string>
#define __STDC_CONSTANT_MACROS
#define SDL_MAIN_HANDLED
extern "C" {
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
#include "libavutil/imgutils.h"
#include "libavutil/samplefmt.h"
#include "libavutil/time.h"
#include "libswresample/swresample.h"
#include "SDL2/SDL.h"
#include "SDL2/SDL_thread.h"
#include "SDL2/SDL_main.h"
#include "SDL2/SDL_video.h"
#include "OpenAL/al.h"
#include "OpenAL/alc.h"
}
using namespace std;
#pragma comment(lib ,"SDL2.lib")
#pragma comment(lib ,"SDL2main.lib")

#define MAX_AUDIO_FARME_SIZE (960000)
#define NUMBUFFERS (4)

#define SFM_REFRESH_EVENT  (SDL_USEREVENT + 1)
#define SFM_BREAK_EVENT  (SDL_USEREVENT + 2)


void initOpenAL(ALuint source) {//初始化
	ALfloat SourceP[] = { 0.0, 0.0, 0.0 };
	ALfloat SourceV[] = { 0.0, 0.0, 0.0 };
	ALfloat ListenerPos[] = { 0.0, 0, 0 };
	ALfloat ListenerVel[] = { 0.0, 0.0, 0.0 };
	ALfloat ListenerOri[] = { 0.0, 0.0, -1.0,  0.0, 1.0, 0.0 };
	alSourcef(source, AL_PITCH, 1.0);
	alSourcef(source, AL_GAIN, 1.0);
	alSourcefv(source, AL_POSITION, SourceP);
	alSourcefv(source, AL_VELOCITY, SourceV);
	alSourcef(source, AL_REFERENCE_DISTANCE, 50.0f);
	alSourcei(source, AL_LOOPING, AL_FALSE);	
}

typedef struct frame {
	void* data;
	int size;
	int samplerate;
	double clock_au;
	int64_t stamp_au;
}Frame, *p_Frame;

int  exit= 0, pause = 0;
int compare(int timeInterval,bool &lead, bool& slower) {
	exit = 0;
	pause = 0;

	while (!exit) {
		if (!pause) {
			SDL_Event event;
			event.type = SFM_REFRESH_EVENT;
			SDL_PushEvent(&event);
		}
		if (lead) {
			std::this_thread::sleep_for(std::chrono::milliseconds(timeInterval)/2);
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(timeInterval));
		}
		if (slower) {
			SDL_Delay(20);
		}
	}
	exit = 0;
	pause = 0;
	SDL_Event event;
	event.type = SFM_BREAK_EVENT;
	SDL_PushEvent(&event);
	return 0;
}



std::queue<p_Frame> packet_qu;
ALuint source;
int64_t stamp_au;
double pts_au;

int videoPlayer(string filePath) {//SDL渲染画面
	AVFormatContext* pformatContext;
	int	i, videoindex;
	AVCodecContext* pcodecContext;
	AVCodec* pcodec;
	AVFrame* pFrame, * pFrameYUV;
	unsigned char* out_buffer;
	AVPacket* packet;
	int ret, get_pic;

	int width, height;
	SDL_Window* screen;
	SDL_Renderer* sdlRenderer;
	SDL_Texture* sdlTexture;
	SDL_Rect sdlRect;
	SDL_Thread* video_tid;
	SDL_Event event;

	struct SwsContext* img_convert_ctx;
	av_register_all();
	avformat_network_init();
	pformatContext = avformat_alloc_context();

	if (avformat_open_input(&pformatContext, filePath.c_str(), NULL, NULL) != 0) {
		cout<<"Couldn't open input stream."<<endl;
		return -1;
	}
	if (avformat_find_stream_info(pformatContext, NULL) < 0) {
		cout<<"Couldn't find stream information."<<endl;
		return -1;
	}
	videoindex = -1;
	for (i = 0; i < pformatContext->nb_streams; i++)
		if (pformatContext->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
			videoindex = i;
			break;
		}
	if (videoindex == -1) {
		cout<<"Didn't find a video stream."<<endl;
		return -1;
	}
	pcodecContext = pformatContext->streams[videoindex]->codec;
	pcodec = avcodec_find_decoder(pcodecContext->codec_id);
	if (pcodec == NULL) {
		cout<<"Codec not found."<<endl;
		return -1;
	}
	if (avcodec_open2(pcodecContext, pcodec, NULL) < 0) {
		cout<<"Could not open codec."<<endl;
		return -1;
	}
	pFrame = av_frame_alloc();
	pFrameYUV = av_frame_alloc();

	out_buffer = (unsigned char*)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_YUV420P, pcodecContext->width, pcodecContext->height, 1));
	av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, out_buffer,
		AV_PIX_FMT_YUV420P, pcodecContext->width, pcodecContext->height, 1);


	cout<<"File Information"<<endl;
	av_dump_format(pformatContext, 0, filePath.c_str(), 0);
	cout << endl;

	img_convert_ctx = sws_getContext(pcodecContext->width, pcodecContext->height, pcodecContext->pix_fmt,
		pcodecContext->width, pcodecContext->height, AV_PIX_FMT_YUV420P, SWS_BICUBIC, NULL, NULL, NULL);


	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
		cout<<"Could not initialize sdl"<<endl;
		return -1;
	}

	width = 1080; height = 720;
	screen = SDL_CreateWindow("Video Player", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		width, height, SDL_WINDOW_OPENGL);

	if (!screen) {
		cout<<"SDL: could not create window "<<endl;
		return -1;
	}
	sdlRenderer = SDL_CreateRenderer(screen, -1, 0);
	sdlTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, pcodecContext->width, pcodecContext->height);
	sdlRect.x = 0;
	sdlRect.y = 0;
	sdlRect.w = width;
	sdlRect.h = height;

	packet = (AVPacket*)av_malloc(sizeof(AVPacket));
	double frameRate = (double)pcodecContext->framerate.num / pcodecContext->framerate.den;

	bool lead = false;
	bool behind= false;

	std::thread refreshThread(compare, (int)(frameRate), std::ref(lead), std::ref(sdlfast));

	double pts_vi = 0;
	double diff = 0;
	while (pts_au >= 0) {
		SDL_WaitEvent(&event);
		if (event.type == SFM_REFRESH_EVENT) {
			while (1) {
				if (av_read_frame(pformatContext, packet) < 0)
					exit = 1;

				if (packet->stream_index == videoindex)
					break;
			}
			ret = avcodec_send_packet(pcodecContext, packet);
			get_pic = avcodec_receive_frame(pcodecContext, pFrame);
			if (ret < 0) {
				cout<<"Decode Error."<<endl;
				return -1;
			}
			if (!get_pic) {
				sws_scale(img_convert_ctx, (const unsigned char* const*)pFrame->data, pFrame->linesize, 0,
					pcodecContext->height, pFrameYUV->data, pFrameYUV->linesize);
			}
			while(1){
				SDL_WaitEvent(&event);
				if (event.type == SFM_REFRESH_EVENT) {
					if (packet_qu.empty()) {
						sws_freeContext(img_convert_ctx);

						SDL_Quit();
						av_frame_free(&pFrameYUV);
						av_frame_free(&pFrame);
						avcodec_close(pcodecContext);
						avformat_close_input(&pformatContext);
					}
					if (true) {
						pts_vi = (double)pFrame->pts * av_q2d(pformatContext->streams[videoindex]->time_base;
						diff = pts_au - pts_vi;
						if (diff > 0.05) {
							lead = true;
						}
						else if(diff <-0.05) {
							lead = false;
							behind= true;
						}
						else {
							lead = false;
							behind= false;
						}
						
					}
					SDL_UpdateTexture(sdlTexture, NULL, pFrameYUV->data[0], pFrameYUV->linesize[0]);
					SDL_RenderClear(sdlRenderer);
					SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, NULL);
					SDL_RenderPresent(sdlRenderer);
					av_free_packet(packet);
					break;
				}
				else if (event.type == SDL_KEYDOWN) {
					if (event.key.keysym.sym == SDLK_SPACE)
						pause = !pause;
				}
				else if (event.type == SDL_QUIT) {
					exit = 1;
				}
				else if (event.type == SFM_BREAK_EVENT) {
					break;
				}
			}
			
		}
	}
	sws_freeContext(img_convert_ctx);
	SDL_Quit();
	av_frame_free(&pFrameYUV);
	av_frame_free(&pFrame);
	avcodec_close(pcodecContext);
	avformat_close_input(&pformatContext);
}

int main(int argc, char* argv[]) {
	
	if (argc != 2) {
		cout << "input error" << endl;
	}
	else {
		string filePath = argv[1];
		//FFmpeg的数据结构
		AVFormatContext* pformatContext; //封装格式上下文结构体，保存了视频文件封装格式相关信息
		AVCodecContext* pcodecContext; //编解码器上下文结构体，保存了视频（音频）编解码相关信息
		AVCodec* pcodec;//每种视频（音频）编解码器(例如H.264解码器)对应一个该结构体
		AVPacket* packet;//用于存储一帧压缩编码数据
		AVFrame* pFrame, *pFrameYUV; //用于存储一帧解码后像素数据
		int index; //编码器索引位置
		uint8_t* out_buffer;	//数据缓冲区
		int out_buffer_size;    //缓冲区大小
		SwrContext* swrCtx;
		double clock_au = 0;
		av_register_all();	//注册库
		avformat_network_init();
		avcodec_register_all();
		pformatContext = avformat_alloc_context();

		//打开视频文件，初始化pformatContext
		if (avformat_open_input(&pformatContext, filePath.c_str(), NULL, NULL) != 0) {
			cout << "Couldn't open input stream." << endl;
			return -1;
		}
		//获取文件信息
		if (avformat_find_stream_info(pformatContext, NULL) < 0) {
			cout << "Couldn't find stream information." << endl;
			return -1;
		}
		index = -1;
		for (int i = 0; i < pformatContext->nb_streams; i++)
			if (pformatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
				index = i;
				break;
			}
		if (index == -1) {
			cout << "Didn't find a video stream." << endl;
			return -1;
		}
		//获取解码器
		pcodec = avcodec_find_decoder(pformatContext->streams[index]->codecpar->codec_id);
		if (pcodec == NULL) {
			cout << "Codec not found." << endl;
			return -1;
		}
		pcodecContext = avcodec_alloc_context3(pcodec);
		avcodec_parameters_to_context(pcodecContext, pformatContext->streams[index]->codecpar);
		pcodecContext->pkt_timebase = pformatContext->streams[index]->time_base;
		//打开解码器
		if (avcodec_open2(pcodecContext, pcodec, NULL) < 0) {
			cout << "Couldn't open codec." << endl;
			return -1;
		}

		//内存分配
		packet = (AVPacket*)av_malloc(sizeof(AVPacket));
		pFrame = av_frame_alloc();
		swrCtx = swr_alloc();

		enum AVSampleFormat in_sample_fmt = pcodecContext->sample_fmt;  //输入的采样格式  
		enum AVSampleFormat out_sample_fmt = AV_SAMPLE_FMT_S16; //输出采样格式16bit PCM  
		int in_sample_rate = pcodecContext->sample_rate; //输入采样率
		int out_sample_rate = in_sample_rate; //输出采样率  
		uint64_t in_ch_layout = pcodecContext->channel_layout; //输入的声道布局   
		uint64_t out_ch_layout = AV_CH_LAYOUT_STEREO; //输出的声道布局
		swr_alloc_set_opts(swrCtx, out_ch_layout, out_sample_fmt, out_sample_rate, in_ch_layout, in_sample_fmt, in_sample_rate, 0, NULL); //设置参数
		swr_init(swrCtx); //初始化
		int out_channel_nb = av_get_channel_layout_nb_channels(out_ch_layout);

		out_buffer = (uint8_t*)av_malloc(MAX_AUDIO_FARME_SIZE);
		int ret;
		while (av_read_frame(pformatContext, packet) >= 0) {
			if (packet->stream_index == index) {
				ret = avcodec_send_packet(pcodecContext, packet);
				if (ret < 0) {
					cout << "avcodec_send_packet：" << ret << endl;
					continue;
				}
				while (ret >= 0) {
					ret = avcodec_receive_frame(pcodecContext, pFrame);
					if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
						break;
					}
					else if (ret < 0) {
						cout << "avcodec_receive_frame：" << AVERROR(ret) << endl;
						return -1;
					}

					if (ret >= 0) {   //frame到输出音频 
						out_buffer = (uint8_t*)av_malloc(MAX_AUDIO_FARME_SIZE);
						swr_convert(swrCtx, &out_buffer, MAX_AUDIO_FARME_SIZE, (const uint8_t**)pFrame->data, pFrame->nb_samples);
						out_buffer_size = av_samples_get_buffer_size(NULL, out_channel_nb, pFrame->nb_samples, out_sample_fmt, 1);
						p_Frame frame = new Frame;
						frame->data = out_buffer;
						frame->size = out_buffer_size;
						frame->samplerate = out_sample_rate;
						clock_au = av_q2d(pcodecContext->time_base) * pFrame->pts;
						frame->clock_au = clock_au;
						packet_qu.push(frame);
					}
				}
			}
			av_packet_unref(packet);
		}

		ALCdevice* pDevice;
		ALCcontext* pContext;

		pDevice = alcOpenDevice(NULL);
		pContext = alcCreateContext(pDevice, NULL);
		alcMakeContextCurrent(pContext);

		if (alcGetError(pDevice) != ALC_NO_ERROR)
			return AL_FALSE;

		ALuint m_buffers[NUMBUFFERS];
		alGenSources(1, &source);
		if (alGetError() != AL_NO_ERROR) {
			cout << "Error generating audio source." << endl;
			return -1;
		}

		initOpenAL(source);
		alDistanceModel(AL_LINEAR_DISTANCE_CLAMPED);
		alListener3f(AL_POSITION, 0, 0, 0);
		alGenBuffers(NUMBUFFERS, buffer);
		ALint processed1 = 0;
		alGetSourcei(source, AL_BUFFERS_PROCESSED, &processed1);

		std::thread sdlplay{ videoPlayer, filePath };
		sdlplay.detach();

		for (int i = 0; i < NUMBUFFERS; i++) {
			if (packet_qu.empty()) return -1;
			PTFRAME frame = packet_qu.front();
			packet_qu.pop();
			if (frame == nullptr)
				return -1;
			alBufferData(bufferID, AL_FORMAT_STEREO16, frame->data, frame->size, frame->samplerate);
			alSourceQueueBuffers(source, 1, &buffer[i]);
			pts_au = frame->clock_au;
			if (frame) {
				av_free(frame->data);
				delete frame;
			}
			return 0;
		}
		alSourcePlay(source);
		while (!packet_qu.empty()) {
			ALint processed = 0;
			alGetSourcei(source, AL_BUFFERS_PROCESSED, &processed);
			while (processed > 0) {
				ALuint bufferID = 0;
				alSourceUnqueueBuffers(source, 1, &bufferID);
				if (packet_qu.empty()) return -1;
				p_Frame frame = packet_qu.front();
				packet_qu.pop();
				if (frame == nullptr)
					return -1;
				alBufferData(bufferID, AL_FORMAT_STEREO16, frame->data, frame->size, frame->samplerate);
				alSourceQueueBuffers(source, 1, &bufferID);
				pts_au = frame->clock_au;
				if (frame) {
					av_free(frame->data);
					delete frame;
				}
				return 0;
				processed--;
			}
			int state;
			alGetSourcei(source, AL_SOURCE_STATE, &state);
			if (state == AL_STOPPED || state == AL_INITIAL) {
				alSourcePlay(source);
			}
			return 0;
		}

		alSourceStop(source);
		alSourcei(source, AL_BUFFER, 0);
		alDeleteBuffers(NUMBUFFERS, buffer);
		alDeleteSources(1, &source);
		av_frame_free(&pFrame);
		swr_free(&swrCtx);


		ALCcontext* pCurContext = alcGetCurrentContext();
		ALCdevice* pCurDevice = alcGetContextsDevice(pCurContext);

		alcMakeContextCurrent(NULL);
		alcDestroyContext(pCurContext);
		alcCloseDevice(pCurDevice);
		return 0;

	}
	
	return 0;
}
