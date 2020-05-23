proj5.exe: main.o random.o SimulationClass.o EventClass.o CustomerClass.o
	g++ main.o random.o SimulationClass.o EventClass.o CustomerClass.o -o proj5.exe

main.o: main.cpp SortedListClass.h FIFOQueueClass.h SimulationClass.h\
	getUserInput.h
	g++ -c main.cpp -o main.o

random.o: random.cpp random.h
	g++ -c random.cpp -o random.o

SimulationClass.o: SimulationClass.cpp SimulationClass.h SortedListClass.h\
	FIFOQueueClass.h random.h EventClass.h CustomerClass.h
	g++ -c SimulationClass.cpp -o SimulationClass.o

EventClass.o: EventClass.cpp EventClass.h
	g++ -c EventClass.cpp -o EventClass.o

CustomerClass.o: CustomerClass.cpp CustomerClass.h
	g++ -c CustomerClass.cpp -o CustomerClass.o

clean:
	rm -rf main.o random.o SimulationClass.o EventClass.o CustomerClass.o proj5.exe

