%clc;
clear; close all;

%% 1. Sprawdzić notacje z użyciem: nawiasów kwadratowych, dwukropka i funkcji przyjmujących wartość macierzy. Sprawdzić działanie poleceń: who, whos, size, clear, save i load
% test_matrix = [1 2 ; 3 4];
% who
% whos
% size(test_matrix)
% filename = "test_file.mat";
% save(filename)
% load(filename)
%%% ctrl + r to comment and ctrl + shift + r to uncomment
%% 2. Zbadać działanie przykładowych operacji (arytmetycznych, arytmetycznych tablicowych, relacji, logicznych) na macierzach. Zbadać polecenia odwołania do fragmentów macierzy
% test_matrix = [1 2 ; 3 4];
% test_matrix2 = [4 3 ; 2 1];
% test_matrix2 * test_matrix
% test_matrix2 .* test_matrix
% test_matrix(1,2)
%% 3. Rozwiązać układ liniowych równań algebraicznych Ax = b. Rozpatrzyć przypadki, w których A jest macierzą kwadratową lub prostokątną. Wykorzystać operator dzielenia macierzy, potęgowanie macierzy (A^(−1)), funkcję wyznaczania macierzy odwrotnej (funkcja inv) lub pseudoinwersji macierzy (funkcja pinv). Wyznaczyć błąd rozwiązania Ax - b.
% A = [2, 1; 1, 3]; % przykładowa macierz kwadratowa
% b = [5; 6];       % wektor b
% x1 = A \ b;           % rozwiązanie za pomocą operatora "\"
% x2 = inv(A) * b      % rozwiązanie za pomocą macierzy odwrotnej
% x3 = A^(-1) * b;      % rozwiązanie przez potęgowanie
% %sprawdzenie
% A*x1-b;
% A*x2-b
% A*x3-b;
%% 4. Wykreślić funkcję y = f (x). Określić funkcję f (x) i dziedzinę X argumentu x ( x należy do X).Korzystając z funkcji plot przygotować M-plik skryptowy, zawierający ciąg poleceń do wykreślenia funkcji f (x) .
% figure;
% x=[-6*pi:0.1:6*pi];
% y=x.*cos(2.*x);
% plot(x,x,'k-')
% hold on
% plot(x,-1.*x,'k-')
% plot(x,y,'b-')
% plot(x,y,'ro')
% grid on
% title('wykres funkcji x*cos(2x)');
%% 5. Wykreślić funkcję z = f (x, y) . Określić funkcję f (x, y) i dziedziny X i Y argumentów x, y ( x należy do X, y należy do Y ). Korzystając z funkcji meshgrid i mesh przygotować M-plik skryptowy, zawierający ciąg poleceń do wykreślenia funkcji f (x, y).
% 
% function dx = rstn (t,x)
%     u=1;
%     dx(1)=x(2);
%     dx(2)=u-5*x(2)-6*x(1);
%     dx=dx';
% end
% 
% function eksperyment(x10,x20)
%     [t x]=ode45(@rstn,[0 5],[x10;x20])
%         % Rysowanie wyników: y' oraz y
%     plot(t, x(:,1), 'r', 'DisplayName', 'y'''); % Pierwsza zmienna - y'
%     hold on;
%     plot(t, x(:,2), 'b', 'DisplayName', 'y');   % Druga zmienna - y
% 
%     % Dodanie legendy
%     legend('show');
%     title('Rozwiązanie układu równań różniczkowych');
%     xlabel('Czas t');
%     ylabel('Stany');
% end
% 
% figure;
% eksperyment(0,0)
% 
% figure;
% eksperyment(0.5,0.3)
% 
% figure;
% eksperyment(-0.2,-0.1)

%% 6. Dla zadanej funkcji przejścia zależnej od parametru a, korzystając z funkcji całkującej ode45, wykreślić rodzinę odpowiedzi układu na pobudzenie skokiem jednostkowym oraz impulsem prostokątnym o amplitudzie 1 i czasie trwania 10. Wyznaczyć równania stanu i równanie wyjścia. Przygotować funkcję wyznaczania pochodnych wektora stanu. Przygotować funkcję wykreślania odpowiedzi układu dla – 11 – podanego wymuszenia, zadanych wartości parametrów a i b. Przygotować M-plik skryptowy, zawierający ciąg poleceń do wykreślania rodziny odpowiedzi układu. Przykłady funkcji przejścia i metod wyznaczania równań stanu: B
%%%nieokreślone warunki początkowe
% function dx = rstn1 (t,x)
%     u=1;
%     dx(1)=(u-x(1))/7;
%     dx(2)=(u-x(2)/5.4);
%     dx=dx';
% end
% 
% function eksperyment1(x10,x20)
%     [t x]=ode45(@rstn1,[0 10],[x10;x20])
%         % Rysowanie wyników: y' oraz y
%     plot(t, x(:,1), 'r'); % Pierwsza zmienna - y'
%     hold on;
%     plot(t, x(:,2), 'b');   % Druga zmienna - y
% 
%     title('Rozwiązanie układu równań różniczkowych');
%     xlabel('Czas t');
%     ylabel('Stany');
% end
% 
% figure;
% eksperyment1(5,5)
% eksperyment1(10,5)
% eksperyment1(15,5)

%% 6. ale z dobrym równaniem co kazał

function dx = rstn (t,x)
    u=1;
    y=x(2)/37.8;
    dx(1)=u-y;
    dx(2)=5*u-12.4*y+x(1);
    dx=dx';
end

function eksperyment(x10,x20)
    [t x]=ode45(@rstn,[0 10],[x10;x20])
        % Rysowanie wyników: y' oraz y
    plot(t, x(:,1), 'r'); % Pierwsza zmienna - y' %możlwie że tu sie pojebało
    hold on;
    plot(t, x(:,2), 'b');   % Druga zmienna - y

    title('Rozwiązanie układu równań różniczkowych');
    xlabel('Czas t');
    ylabel('Stany');
end

figure;
eksperyment(5,5)
eksperyment(10,5)
eksperyment(15,5)
grid on;

%% 7. Korzystając z pakietu Simulink zbudować modele wybranego układu o funkcji przejścia i wymuszeniach opisanych w punkcie 6, wykorzystując bloki:  elementarne (Intergrator, Sum i Gain)  funkcji przejścia (Transfer Fcn),  równań stanu (State-Space). Przygotować funkcję wykreślania odpowiedzi układu dla podanego wymuszenia i dla zadanego parametru a. Przygotować M-plik skryptowy, zawierający ciąg poleceń do wykreślania rodziny odpowiedzi układu.

%uklad03;
figure;
hold on;

opt = simset('InitialState' , [5 5]);
[t1 x y1] = sim ('uklad03', [0 10], opt);

plot(t1, y1(:,1), 'r');
plot(t1, y1(:,2), 'b');

opt = simset('InitialState' , [10 5]);
[t2 x y2] = sim ('uklad03', [0 10], opt);

plot(t2, y2(:,1), 'r');
plot(t2, y2(:,2), 'b');

opt = simset('InitialState' , [15 5]);
[t3 x y3] = sim ('uklad03', [0 10], opt);

plot(t3, y3(:,1), 'r');
plot(t3, y3(:,2), 'b');
grid on;
title('Wykresy dla różnych stanów początkowych ale zrobione Simulinkiem');
xlabel('Czas (s)');
ylabel('Wyjście');

%% 8. zadanie chyba z pizdy nie robiłem jak trzeba to pozdro