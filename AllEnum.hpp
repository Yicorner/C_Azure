#ifndef ALL_ENUM_HPP
#define ALL_ENUM_HPP

enum BodyLocation {
	NOT_DETECTED = 0,  // δʶ��
	HEAD = 1,          // ͷ��
	CHEST = 2,         // �ز�
	ABDOMEN = 3        // ����
};
enum State {
	DYNAMIC = 0,  // ��̬
	FROM_SMAPLE = 1,// ��GetSample�л�ȡ���ݣ��ǵü���
	FROM_FILE = 2 // ���ļ��л�ȡ���ݣ��ǵü���
};
enum RealTimeDisplayState {
	ONLY_COLOR = 0, // ֻ��ʾ��ɫͼ
	ONLY_DEPTH = 1, // ֻ��ʾ���ͼ
	COLOR_DEPTH = 2 // ͬʱ��ʾ��ɫͼ�����ͼ
};
#endif // ALLENUM_HPP