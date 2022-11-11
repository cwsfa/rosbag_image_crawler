import os
import re
import pandas as pd
import glob

USERNAME = os.getlogin()


def set_dir(csv_path, file):
    """Set directory for the csv file."""
    file_stripped = file.strip('/')
    dir = csv_path + '/' + file_stripped
    csv_name = file_stripped.split('.')[0]
    # if the file exists, return unique name
    if os.path.exists(dir):
        files_list = [i.split('/')[-1] for i in glob.glob(
            csv_path + '/' + csv_name + '*.csv')[1:]]
        try:
            last = sorted(files_list, key=lambda x: int(split(x, 1)))[-1]
            new = int(split(last, 1)) + 1
        except IndexError:
            new = 1
        finally:
            return csv_path + '/' + csv_name + '_' + str(new) + '.csv'
    # if the file doesn't exist, return dir
    return dir


def split(string, index, separtors='[\. _]'):
    """Split a given string by given separators."""
    return re.split(separtors, string)[index]


def main(csv_name='labels.csv'):
    """Save a label csv file to the output_dir."""
    try:
        output_dir = '/home/' + USERNAME + '/clawed_image_label'
        frame_list = os.listdir(output_dir + '/color')
        # get sorted frame name list
        sorting_key = lambda x: int(x.split('_')[0])
        frame_list_sorted = sorted(frame_list, key=sorting_key)
        # prepare list for data frame
        data_list = [[int(split(each, 0)),
                      int(split(each, 1))]
                      for each in frame_list_sorted]
        # change data list to dataframe
        df = pd.DataFrame(data_list)
        # rename dataframe columns
        df = df.rename(columns={0:'image', 1:'label'})
        # set csv name
        csv_dir = set_dir(output_dir, csv_name)
        # save csv to output_dir
        df.to_csv(csv_dir, index=False)
    except FileNotFoundError as e:
        print(e)
    else:
        print('Error running get_label_csv.py')


if __name__ == '__main__':
    main()
