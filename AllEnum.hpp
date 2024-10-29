#ifndef ALL_ENUM_HPP
#define ALL_ENUM_HPP

enum BodyLocation {
	NOT_DETECTED = 0,  // 未识别到
	HEAD = 1,          // 头部
	CHEST = 2,         // 胸部
	ABDOMEN = 3        // 肚子
};
enum State {
	DYNAMIC = 0,  // 动态
	FROM_SMAPLE = 1,// 从GetSample中获取数据，记得加锁
	FROM_FILE = 2 // 从文件中获取数据，记得加锁
};

#endif // ALLENUM_HPP