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

#endif // ALLENUM_HPP