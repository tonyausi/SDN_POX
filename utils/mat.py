import scipy.io

def f_loat_mat(matfile):
    try:
        mat = scipy.io.loadmat(matfile)
        return mat
    except IOError:
        print 'fail to load mat file: %s and return None' %matfile
        return None
        
if __name__ == "__main__":
    mat_in = f_loat_mat('USNET_2.mat')
    print mat_in.keys()
    print mat_in['RM']