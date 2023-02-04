#!/bin/bash
cd ../build
cmake .. && make
cd ../scripts

# COMMAND LINE ARGUMENTS CHECK #
if [[ $# -ne 9 && $# -ne 11 && $# -ne 15 ]]
then
    echo "Usage: ./run_test_cases -i <test_cases_folder> 
            -algorithm <local_search/simulated_annealing/ant_colony> 
            -init_algo <onion/ch2poly | except ant_colony>
            -L [L parameter according to algorithm]
            -max [maximal area polygonization]
            -min [minimal area polygonization]
            -threshold <double> [in local search]
            -annealing <local/global/subdivision in simulated annealing>
            -alpha <double> -beta <double> -rho <double> -elitism <0 or 1>
            [alpha, beta, rho, elitism only in ant_colony algorithm]"
    exit
fi

OUT_FILE=""

while :; do
    case $1 in
        -i) 
            TESTS_FOLDER=$2
            shift
            ;;
        -o)       
            OUT_FILE=$2
            shift
            ;;
        -algorithm)
            ALGORITHM=$2
            shift
            ;;
        -init_algo)
            INIT_ALGO=$2
            shift
            ;;
        -L)         
            L=$2
            shift
            ;;
        -min)
            OPTIMIZATION=$1
            #shift
            ;;
        -max)
            OPTIMIZATION=$1
            #shift
            ;;
        -threshold)    
            THRESHOLD=$2
            shift
            ;;
        -annealing)
            ANNEALING=$2
            shift
            ;;
        -alpha)    
            ALPHA=$2
            shift
            ;;
        -beta)    
            BETA=$2
            shift
            ;;
        -rho)    
            RHO=$2
            shift
            ;;
        -elitism)    
            ELITISM=$2
            shift
            ;;
        *)               # Default case: No more options, so break out of the loop.
            break
    esac
    shift
done

function ProgressBar {
	let _progress=(${1}*100/${2}*100)/100
	let _done=(${_progress}*4)/10
	let _left=40-$_done
	_done=$(printf "%${_done}s")
	_left=$(printf "%${_left}s")
    printf "\rProgress : [${_done// /#}${_left// /-}] ${_progress}%%\n"
}

case $ALGORITHM in

  local_search)
    if [[ $INIT_ALGO != "" ]]
    then
        algo_cmd="-o output -algorithm $ALGORITHM -init_algo $INIT_ALGO -L $L $OPTIMIZATION -threshold $THRESHOLD"
    else 
        algo_cmd="-o output -algorithm $ALGORITHM -L $L $OPTIMIZATION -threshold $THRESHOLD"
    fi
    ;;

  simulated_annealing)
    if [[ $INIT_ALGO != "" ]]
    then
        algo_cmd="-o output -algorithm $ALGORITHM -init_algo $INIT_ALGO -L $L $OPTIMIZATION -annealing $ANNEALING"
    else
        algo_cmd="-o output -algorithm $ALGORITHM -L $L $OPTIMIZATION -annealing $ANNEALING"
    fi
    ;;

  ant_colony)
    algo_cmd="-o output -algorithm $ALGORITHM -L $L $OPTIMIZATION -alpha $ALPHA -beta $BETA -rho $RHO -elitism $ELITISM"
    ;;

  *)
    echo "Invalid algorithm name!"
    exit
    ;;
esac

num_of_tests=$(ls $TESTS_FOLDER | wc -l)
echo "No of test to check: $num_of_tests"

count=1
all_passed=1
TIMEFORMAT=%R

if [[ $ALGORITHM == ant_colony ]]
then
    echo "ACO will run for at most 20 points due to memory constraints ..."
fi

for input_file in $TESTS_FOLDER/*
do
    if [[ $input_file == *"uniform"* ]]
    then
        idx=8
    else
        idx=7
    fi
    # extract number of points from input_file 
    point_num=$(sed -n '1p' $input_file | tr -d '(,)'| awk '{ print $'"$idx"' }')    
    if [[ $point_num -gt 20 && $ALGORITHM == "ant_colony" ]] 
    then
        exit
    fi
    # user_time="$(time ( ../build/bin/optimal_polygon -i $input_file $algo_cmd ) 2>&1 1>/dev/null )"
    
    ../build/bin/optimal_polygon -t -i $input_file $algo_cmd

    res_point_num=$(sed -n '1p' optimal_results.txt | awk '{ print $1 }')
    is_simple=$(sed -n '1p' optimal_results.txt | awk '{ print $2 }')

    test_case_passed=1
    if [[ $point_num != $res_point_num ]]
    then
        printf "$input_file  --Different number of points!\n"
        printf "Num of points found: $res_point_num \t True points num $point_num\n"
        test_case_passed=0
    fi

    if [[ $is_simple != 'Y' ]]
    then
        printf "$input_file  --Polygon is not simple!\n"
        test_case_passed=0
    fi

    if [[ $test_case_passed -ne 1 ]] 
    then
        echo "$(tput setaf 1)ATTENTION!$ $(tput sgr0)TEST '$input_file' FAILED!"
        all_passed=0
        exit
    else
        echo "$(tput setaf 2)OK$(tput sgr0) TEST CASE '$input_file' PASSED"
        if [[ $OUT_FILE != "" ]]; then
            echo $point_num $user_time >> $OUT_FILE
        fi
    fi

	ProgressBar ${count} ${num_of_tests}
    count=$(( $count + 1 ))
done

if [[ $all_passed -eq 1 ]]
then
    echo "All test cases passed!"
else 
    echo "Some test case(s) failed"
fi

# Delete the results file used for testing 
rm -f optimal_results.txt