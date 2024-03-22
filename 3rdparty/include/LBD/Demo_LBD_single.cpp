#include "LineMatchingAlgorithm.hpp"

using namespace std;

void usage(int argc, char **argv)
{
	cout << "Usage: " << argv[0] << "  image1.png"
		 << "  image2.png" << " out.png" << endl;
}

int main(int argc, char **argv)
{
//	int ret = -1;
//	if (argc < 4)
//	{
//		usage(argc, argv);
//		return ret;
//	}
//    //load first image from file
//	cout<<"image process"<<endl;
//	cout<<"argv[1]="<<argv[1]<<endl;
//	cout<<"argv[2]="<<argv[2]<<endl;
//	cout<<"argv[3]="<<argv[3]<<endl;
//	string 
//	test(argv[1],argv[2],argv[3],"",false);
	int i=0;
	while(true)
	{
		cout<<"========================================="<<endl;
		cout<<i<<endl;
		image_process("/media/sun/Elements/sun/rgbd_dataset_freiburg3_cabinet/rgb/1341841280.314599.png",
					  "/media/sun/Elements/sun/rgbd_dataset_freiburg3_cabinet/rgb/1341841279.982535.png","out.png","",false);
		i++;
	}
    return 0;
}
